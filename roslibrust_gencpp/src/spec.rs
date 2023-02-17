use roslibrust_codegen::{FieldInfo, MessageFile};
use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize, Clone, Debug)]
pub struct Field {
    pub name: String,
    pub field_type: String,
    pub package: Option<String>,
}

impl From<&FieldInfo> for Field {
    fn from(value: &FieldInfo) -> Self {
        Self {
            name: value.field_name.clone(),
            field_type: value.field_type.field_type.clone(),
            package: value.field_type.package_name.clone(),
        }
    }
}

#[derive(Deserialize, Serialize, Clone, Debug)]
pub struct MessageSpecification {
    pub short_name: String,
    pub package: String,
    pub fields: Vec<Field>,
    pub md5sum_first: String,
    pub md5sum_second: String,
    pub description: String,
    pub is_fixed_length: bool,
}

impl MessageSpecification {
    pub fn from_data(data: &MessageFile) -> Self {
        let md5sum = data.get_md5sum();
        let mut spec = Self {
            short_name: data.get_short_name(),
            package: data.get_package_name(),
            fields: data.get_fields().iter().map(Field::from).collect(),
            md5sum_first: String::from(&md5sum[..md5sum.len() / 2]),
            md5sum_second: String::from(&md5sum[(md5sum.len() / 2)..]),
            description: String::from("Not a real description"),
            is_fixed_length: data.is_fixed_length(),
        };
        spec.fields.iter_mut().for_each(|field| {
            if field.package.is_none() {
                field.package = Some(data.get_package_name());
            }
        });
        spec
    }
}
