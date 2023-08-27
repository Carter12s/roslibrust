use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use std::io::{Cursor, Read, Write};

pub struct ConnectionHeader {
    pub caller_id: String,
    pub latching: bool,
    pub msg_definition: String,
    pub md5sum: String,
    pub topic: String,
    pub topic_type: String,
    pub tcp_nodelay: bool,
}

impl ConnectionHeader {
    pub fn from_bytes(header_data: &[u8]) -> std::io::Result<ConnectionHeader> {
        let mut cursor = Cursor::new(header_data);
        let header_length = cursor.read_u32::<LittleEndian>()?;
        if header_length as usize > header_data.len() {
            return Err(std::io::ErrorKind::InvalidInput.into());
        }

        let mut msg_definition = String::new();
        let mut caller_id = String::new();
        let mut latching = false;
        let mut md5sum = String::new();
        let mut topic = String::new();
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
                field[equals_pos + 1..].clone_into(&mut topic);
            } else if field.starts_with("type=") {
                field[equals_pos + 1..].clone_into(&mut topic_type);
            } else if field.starts_with("tcp_nodelay=") {
                let mut tcp_nodelay_str = String::new();
                field[equals_pos + 1..].clone_into(&mut tcp_nodelay_str);
                tcp_nodelay = &tcp_nodelay_str != "0";
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
            topic_type,
            tcp_nodelay,
        })
    }

    pub fn to_bytes(&self, to_publisher: bool) -> std::io::Result<Vec<u8>> {
        let mut header_data = Vec::with_capacity(1024);
        // Start by skipping the length header since we don't know yet
        header_data.write_u32::<LittleEndian>(0)?;

        let caller_id_str = format!("caller_id={}", self.caller_id);
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

        let topic = format!("topic={}", self.topic);
        header_data.write_u32::<LittleEndian>(topic.len() as u32)?;
        header_data.write(topic.as_bytes())?;

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
