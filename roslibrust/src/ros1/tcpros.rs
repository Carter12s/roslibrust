use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use log::*;
use std::io::{Cursor, Read, Write};
use tokio::net::TcpStream;

use super::names::Name;

// Implementation of ConnectionHeader is based off of ROS documentation here:
// https://wiki.ros.org/ROS/Connection%20Header
// and here:
// https://wiki.ros.org/ROS/TCPROS
#[derive(Clone, Debug)]

pub struct ConnectionHeader {
    pub caller_id: String,
    pub latching: bool, // TODO this field should be optional and None for service clients and servers
    pub msg_definition: String, // TODO this should be optional and None for service clients and servers
    pub md5sum: Option<String>,
    // TODO we may want to distinguish between service and topic headers with different types?
    pub service: Option<String>,
    pub topic: Option<String>,
    pub topic_type: String,
    pub tcp_nodelay: bool, // TODO this field should be optional and None for service clients and servers
    pub persistent: Option<bool>,
    // TODO service server only has to respond with caller_id (all other fields optional)
}

impl ConnectionHeader {
    /// Parses a connection header from a byte array
    /// This specifically expects to start at the first byte of the first field
    /// of the header and bypass the bytes representing the length of the header
    pub fn from_bytes(header_data: &[u8]) -> std::io::Result<ConnectionHeader> {
        let mut cursor = Cursor::new(header_data);

        let mut msg_definition = String::new();
        let mut caller_id = String::new();
        let mut latching = false;
        let mut md5sum = None;
        let mut topic = None;
        let mut service = None;
        let mut topic_type = String::new();
        let mut tcp_nodelay = false;
        let mut persistent = None;

        // TODO: Unhandled: error, persistent
        while cursor.position() < header_data.len() as u64 {
            let field_length = cursor.read_u32::<LittleEndian>()? as usize;
            let mut field = vec![0u8; field_length];
            cursor.read_exact(&mut field)?;
            let field = String::from_utf8(field).map_err(|e| {
                warn!("Failed to parse field in connection header as valid utf8: {e:#?}, Full header: {header_data:#?}");
                std::io::ErrorKind::InvalidData
            })?;
            let equals_pos = match field.find('=') {
                Some(pos) => pos,
                None => continue,
            };
            if field.starts_with("message_definition=") {
                field[equals_pos + 1..].clone_into(&mut msg_definition);
            } else if field.starts_with("callerid=") {
                field[equals_pos + 1..].clone_into(&mut caller_id);
            } else if field.starts_with("latching=") {
                let mut latching_str = String::new();
                field[equals_pos + 1..].clone_into(&mut latching_str);
                latching = &latching_str != "0";
            } else if field.starts_with("md5sum=") {
                let mut md5sum_str = String::new();
                field[equals_pos + 1..].clone_into(&mut md5sum_str);
                md5sum = Some(md5sum_str);
            } else if field.starts_with("topic=") {
                let mut topic_str = String::new();
                field[equals_pos + 1..].clone_into(&mut topic_str);
                topic = Some(topic_str);
            } else if field.starts_with("service=") {
                let mut service_str = String::new();
                field[equals_pos + 1..].clone_into(&mut service_str);
                service = Some(service_str);
            } else if field.starts_with("type=") {
                field[equals_pos + 1..].clone_into(&mut topic_type);
            } else if field.starts_with("tcp_nodelay=") {
                let mut tcp_nodelay_str = String::new();
                field[equals_pos + 1..].clone_into(&mut tcp_nodelay_str);
                tcp_nodelay = &tcp_nodelay_str != "0";
            } else if field.starts_with("persistent=") {
                let mut persistent_str = String::new();
                field[equals_pos + 1..].clone_into(&mut persistent_str);
                persistent = Some(&persistent_str != "0");
            } else if field.starts_with("probe=") {
                // probe is apprantly an undocumented header field that is sent
                // by certain ros tools when they initiate a service_client connection to a service server
                // for the purpose of discovering the service type
                // If you do `rosservice call /my_service` and hit TAB you'll see this field in the connection header
                // we can ignore it
            } else if field.starts_with("response_type=") || field.starts_with("request_type=") {
                // More undocumented fields!
                // Discovered in testing that some roscpp service servers will set these on service responses
                // We can ignore em
            } else if field.starts_with("error=") {
                log::error!("Error reported in TCPROS connection header: {field}, full header: {header_data:#?}");
            } else {
                log::warn!("Encountered unhandled field in connection header: {field}, full header: {header_data:#?}");
            }
        }

        let header = ConnectionHeader {
            caller_id,
            latching,
            msg_definition,
            md5sum,
            topic,
            service,
            topic_type,
            tcp_nodelay,
            persistent,
        };
        trace!(
            "Got connection header: {header:?} for topic {:?}",
            header.topic
        );
        Ok(header)
    }

    /// Serializes the connection header to a byte array
    /// to_publisher (currently) controls whether or not tcp_nodelay is included
    /// we should probably change that
    pub fn to_bytes(&self, to_publisher: bool) -> std::io::Result<Vec<u8>> {
        let mut header_data = Vec::with_capacity(1024);
        // Start by skipping the length header since we don't know yet
        header_data.write_u32::<LittleEndian>(0)?;

        let caller_id_str = format!("callerid={}", self.caller_id);
        header_data.write_u32::<LittleEndian>(caller_id_str.len() as u32)?;
        header_data.write(caller_id_str.as_bytes())?;

        let latching_str = format!("latching={}", if self.latching { 1 } else { 0 });
        header_data.write_u32::<LittleEndian>(latching_str.len() as u32)?;
        header_data.write(latching_str.as_bytes())?;

        if let Some(md5sum) = self.md5sum.as_ref() {
            let md5sum = format!("md5sum={}", md5sum);
            header_data.write_u32::<LittleEndian>(md5sum.len() as u32)?;
            header_data.write(md5sum.as_bytes())?;
        }

        let msg_definition = format!("message_definition={}", self.msg_definition);
        header_data.write_u32::<LittleEndian>(msg_definition.len() as u32)?;
        header_data.write(msg_definition.as_bytes())?;

        if to_publisher {
            let tcp_nodelay = format!("tcp_nodelay={}", if self.tcp_nodelay { 1 } else { 0 });
            header_data.write_u32::<LittleEndian>(tcp_nodelay.len() as u32)?;
            header_data.write(tcp_nodelay.as_bytes())?;
        }

        if let Some(topic) = self.topic.as_ref() {
            let topic = format!("topic={}", topic);
            header_data.write_u32::<LittleEndian>(topic.len() as u32)?;
            header_data.write(topic.as_bytes())?;
        }

        if let Some(service) = self.service.as_ref() {
            let service = format!("service={}", service);
            header_data.write_u32::<LittleEndian>(service.len() as u32)?;
            header_data.write(service.as_bytes())?;
        }

        let topic_type = format!("type={}", self.topic_type);
        header_data.write_u32::<LittleEndian>(topic_type.len() as u32)?;
        header_data.write(topic_type.as_bytes())?;

        if let Some(persistent) = self.persistent {
            let persistent = format!("persistent={}", if persistent { 1 } else { 0 });
            header_data.write_u32::<LittleEndian>(persistent.len() as u32)?;
            header_data.write(persistent.as_bytes())?;
        }

        // Now that we know the length, stick its value in the first 4 bytes
        let total_length = (header_data.len() - 4) as u32;
        for (idx, byte) in total_length.to_le_bytes().iter().enumerate() {
            header_data[idx] = *byte;
        }

        Ok(header_data)
    }
}

/// Creates a new TCP connection to the given server URI and sends the connection header.
/// The only current user of this is service clients.
pub async fn establish_connection(
    node_name: &Name,
    topic_name: &str,
    server_uri: &str,
    conn_header: ConnectionHeader,
) -> Result<TcpStream, std::io::Error> {
    use tokio::io::AsyncWriteExt;

    // Okay in Shane's version of this the server_uri is coming in as "rosrpc://localhost:41105"
    // which is causing this to break...
    // Not sure what that is the fault of, but I'm going to try to clean up the address here
    // I think the correct way of doing this would be with some king of URI parsing library, for now being very lazy
    // TODO confirm if all libs respond with rosrpc:// or not...
    let server_uri = &server_uri.replace("rosrpc://", "");

    let mut stream = TcpStream::connect(server_uri).await.map_err(
        |err| {
            log::error!(
                "Failed to establish TCP connection to server {server_uri} for topic {topic_name}: {err}"
            );
            err
        },
    )?;

    // Write our own connection header to the stream
    let conn_header_bytes = conn_header.to_bytes(true)?;
    trace!(
        "Sending connection header to server {server_uri} for topic {topic_name}: {conn_header:?}"
    );
    stream.write_all(&conn_header_bytes[..]).await?;

    // Recieve the header from the server
    let responded_header = receive_header(&mut stream).await;
    if let Ok(_responded_header) = responded_header {
        // TODO we should really examine this md5sum logic...
        // according to the ROS documentation, the service isn't required to respond
        // with anything other than caller_id
        // if conn_header.md5sum != responded_header.md5sum {
        //     log::error!(
        //         "Tried to connect to {node_name} for {topic_name}, but md5sums do not match. Expected {}, received {}",
        //         conn_header.md5sum,
        //         responded_header.md5sum
        //     );
        //     return Err(std::io::ErrorKind::InvalidData.into());
        // }

        log::debug!(
            "Established connection with node {node_name} for topic {:?}",
            conn_header.topic
        );
        Ok(stream)
    } else {
        log::error!("Could not parse connection header data sent by server");
        Err(std::io::ErrorKind::InvalidData)
    }
    .map_err(std::io::Error::from)
}

// Reads a complete ROS connection header from the given stream
pub async fn receive_header(stream: &mut TcpStream) -> Result<ConnectionHeader, std::io::Error> {
    // Bring trait def into scope
    use tokio::io::AsyncReadExt;
    // Recieve the header length
    let mut header_len_bytes = [0u8; 4];
    let _num_bytes_read = stream.read_exact(&mut header_len_bytes).await?;
    // This is the length of the header itself
    let header_len = u32::from_le_bytes(header_len_bytes) as usize;

    // Initialize a buffer to hold the header
    let mut header_bytes = vec![0u8; header_len];
    let _num_bytes_read = stream.read_exact(&mut header_bytes).await?;
    ConnectionHeader::from_bytes(&header_bytes)
}

/// Reads the body of a message from the given stream
/// It first reads the length of the body, then reads the body itself
/// The returned Vec<> includes the length of the body at the front as serde_rosmsg expects
pub async fn receive_body(stream: &mut TcpStream) -> Result<Vec<u8>, std::io::Error> {
    // Bring trait def into scope
    use tokio::io::AsyncReadExt;

    // Read the four bytes of size directly
    let mut body_len_bytes = [0u8; 4];
    stream.read_exact(&mut body_len_bytes).await?;
    let body_len = u32::from_le_bytes(body_len_bytes);
    trace!("Read length from stream: {}", body_len);

    // Allocate buffer space for length and body
    let mut body = vec![0u8; body_len as usize + 4];
    // Copy the length into the first four bytes
    body[..4].copy_from_slice(&body_len.to_le_bytes());
    // Read the body into the buffer after the header
    stream.read_exact(&mut body[4..]).await?;
    trace!("Read body of size: {}", body.len());

    // Return body
    Ok(body)
}

#[cfg(test)]
mod test {
    use super::ConnectionHeader;

    // From ROS website: http://wiki.ros.org/ROS/Connection%20Header
    #[test_log::test]
    fn ros_example_header() {
        let bytes: Vec<u8> = vec![
            0x20, 0x00, 0x00, 0x00, 0x6d, 0x65, 0x73, 0x73, 0x61, 0x67, 0x65, 0x5f, 0x64, 0x65,
            0x66, 0x69, 0x6e, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x3d, 0x73, 0x74, 0x72, 0x69, 0x6e,
            0x67, 0x20, 0x64, 0x61, 0x74, 0x61, 0x0a, 0x0a, 0x25, 0x00, 0x00, 0x00, 0x63, 0x61,
            0x6c, 0x6c, 0x65, 0x72, 0x69, 0x64, 0x3d, 0x2f, 0x72, 0x6f, 0x73, 0x74, 0x6f, 0x70,
            0x69, 0x63, 0x5f, 0x34, 0x37, 0x36, 0x37, 0x5f, 0x31, 0x33, 0x31, 0x36, 0x39, 0x31,
            0x32, 0x37, 0x34, 0x31, 0x35, 0x35, 0x37, 0x0a, 0x00, 0x00, 0x00, 0x6c, 0x61, 0x74,
            0x63, 0x68, 0x69, 0x6e, 0x67, 0x3d, 0x31, 0x27, 0x00, 0x00, 0x00, 0x6d, 0x64, 0x35,
            0x73, 0x75, 0x6d, 0x3d, 0x39, 0x39, 0x32, 0x63, 0x65, 0x38, 0x61, 0x31, 0x36, 0x38,
            0x37, 0x63, 0x65, 0x63, 0x38, 0x63, 0x38, 0x62, 0x64, 0x38, 0x38, 0x33, 0x65, 0x63,
            0x37, 0x33, 0x63, 0x61, 0x34, 0x31, 0x64, 0x31, 0x0e, 0x00, 0x00, 0x00, 0x74, 0x6f,
            0x70, 0x69, 0x63, 0x3d, 0x2f, 0x63, 0x68, 0x61, 0x74, 0x74, 0x65, 0x72, 0x14, 0x00,
            0x00, 0x00, 0x74, 0x79, 0x70, 0x65, 0x3d, 0x73, 0x74, 0x64, 0x5f, 0x6d, 0x73, 0x67,
            0x73, 0x2f, 0x53, 0x74, 0x72, 0x69, 0x6e, 0x67,
        ];

        let header = ConnectionHeader::from_bytes(&bytes).unwrap();
        assert_eq!(header.msg_definition, "string data\n\n");
        assert_eq!(header.caller_id, "/rostopic_4767_1316912741557");
        assert_eq!(header.latching, true);
        assert_eq!(header.topic, Some("/chatter".to_owned()));
        assert_eq!(header.topic_type, "std_msgs/String");
    }

    #[test_log::test]
    fn example_from_testing() {
        // example taken from `rostopic echo` with our ros1_talker example
        let bytes: Vec<u8> = vec![
            37, 0, 0, 0, 99, 97, 108, 108, 101, 114, 105, 100, 61, 47, 114, 111, 115, 116, 111,
            112, 105, 99, 95, 49, 49, 54, 56, 95, 49, 55, 50, 48, 50, 49, 53, 56, 51, 56, 57, 48,
            50, 39, 0, 0, 0, 109, 100, 53, 115, 117, 109, 61, 57, 57, 50, 99, 101, 56, 97, 49, 54,
            56, 55, 99, 101, 99, 56, 99, 56, 98, 100, 56, 56, 51, 101, 99, 55, 51, 99, 97, 52, 49,
            100, 49, 31, 0, 0, 0, 109, 101, 115, 115, 97, 103, 101, 95, 100, 101, 102, 105, 110,
            105, 116, 105, 111, 110, 61, 115, 116, 114, 105, 110, 103, 32, 100, 97, 116, 97, 10,
            13, 0, 0, 0, 116, 99, 112, 95, 110, 111, 100, 101, 108, 97, 121, 61, 48, 14, 0, 0, 0,
            116, 111, 112, 105, 99, 61, 47, 99, 104, 97, 116, 116, 101, 114, 20, 0, 0, 0, 116, 121,
            112, 101, 61, 115, 116, 100, 95, 109, 115, 103, 115, 47, 83, 116, 114, 105, 110, 103,
        ];

        let header = ConnectionHeader::from_bytes(&bytes).unwrap();
        assert_eq!(header.caller_id, "/rostopic_1168_1720215838902");
        assert_eq!(header.topic_type, "std_msgs/String");
        assert_eq!(
            header.md5sum,
            Some("992ce8a1687cec8c8bd883ec73ca41d1".to_string())
        );
    }
}
