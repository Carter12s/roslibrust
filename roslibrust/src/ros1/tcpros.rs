use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
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
    pub latching: bool,
    pub msg_definition: String,
    pub md5sum: String,
    // TODO we may want to distinguish between service and topic headers with different types?
    pub service: Option<String>,
    pub topic: Option<String>,
    pub topic_type: String,
    pub tcp_nodelay: bool,
}

impl ConnectionHeader {
    pub fn from_bytes(header_data: &[u8]) -> std::io::Result<ConnectionHeader> {
        let mut cursor = Cursor::new(header_data);

        let mut msg_definition = String::new();
        let mut caller_id = String::new();
        let mut latching = false;
        let mut md5sum = String::new();
        let mut topic = None;
        let mut service = None;
        let mut topic_type = String::new();
        let mut tcp_nodelay = false;

        // TODO: Unhandled: error, persistent

        while cursor.position() < header_data.len() as u64 {
            let field_length = cursor.read_u32::<LittleEndian>()? as usize;
            let mut field = vec![0u8; field_length];
            cursor.read_exact(&mut field)?;
            let field = String::from_utf8(field).unwrap();
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
                field[equals_pos + 1..].clone_into(&mut md5sum);
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
            } else if field.starts_with("error=") {
                log::error!("Error reported in TCPROS connection header: {field}");
            } else {
                log::warn!("Encountered unhandled field in connection header: {field}");
            }
        }

        Ok(ConnectionHeader {
            caller_id,
            latching,
            msg_definition,
            md5sum,
            topic,
            service,
            topic_type,
            tcp_nodelay,
        })
    }

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

        let md5sum = format!("md5sum={}", self.md5sum);
        header_data.write_u32::<LittleEndian>(md5sum.len() as u32)?;
        header_data.write(md5sum.as_bytes())?;

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
    use tokio::io::{AsyncReadExt, AsyncWriteExt};

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

    let conn_header_bytes = conn_header.to_bytes(true)?;
    stream.write_all(&conn_header_bytes[..]).await?;

    let mut header_len_bytes = [0u8; 4];
    let _header_bytes = stream.read_exact(&mut header_len_bytes).await?;
    let header_len = u32::from_le_bytes(header_len_bytes) as usize;

    let mut responded_header_bytes = Vec::with_capacity(header_len);
    let bytes = stream.read_buf(&mut responded_header_bytes).await?;
    if let Ok(responded_header) = ConnectionHeader::from_bytes(&responded_header_bytes[..bytes]) {
        if conn_header.md5sum == responded_header.md5sum {
            log::debug!(
                "Established connection with node {node_name} for topic {:?}",
                conn_header.topic
            );
            Ok(stream)
        } else {
            log::error!(
                "Tried to connect to {node_name} for {topic_name}, but md5sums do not match. Expected {}, received {}",
                conn_header.md5sum,
                responded_header.md5sum
            );
            Err(std::io::ErrorKind::InvalidData)
        }
    } else {
        log::error!("Could not parse connection header data sent by server");
        Err(std::io::ErrorKind::InvalidData)
    }
    .map_err(std::io::Error::from)
}
