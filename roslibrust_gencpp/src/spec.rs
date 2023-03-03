use roslibrust_codegen::{ConstantInfo, FieldInfo, MessageFile, ServiceFile};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

lazy_static::lazy_static! {
    pub static ref ROS_TYPE_TO_CPP_TYPE_MAP: HashMap<&'static str, &'static str> = vec![
        ("bool", "bool"),
        ("int8", "int8_t"),
        ("uint8", "uint8_t"),
        ("byte", "uint8_t"),
        ("char", "char"),
        ("int16", "int16_t"),
        ("uint16", "uint16_t"),
        ("int32", "int32_t"),
        ("uint32", "uint32_t"),
        ("int64", "int64_t"),
        ("uint64", "uint64_t"),
        ("float32", "float"),
        ("float64", "double"),
        ("string", "::std::string"),
        ("time", "::ros::Time"),
        ("duration", "::ros::Duration"),
    ].into_iter().collect();
}

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

impl Field {
    pub fn is_intrinsic_type(&self) -> bool {
        ROS_TYPE_TO_CPP_TYPE_MAP.contains_key(self.field_type.as_str())
    }
}

#[derive(Deserialize, Serialize, Clone, Debug)]
pub struct Constant {
    pub name: String,
    pub constant_type: String,
    pub constant_value: String,
}

impl From<&ConstantInfo> for Constant {
    fn from(value: &ConstantInfo) -> Self {
        Self {
            name: value.constant_name.clone(),
            constant_type: value.constant_type.clone(),
            constant_value: value.constant_value.to_string(),
        }
    }
}

#[derive(Deserialize, Serialize, Clone, Debug)]
pub struct MessageSpecification {
    pub short_name: String,
    pub package: String,
    pub fields: Vec<Field>,
    pub constants: Vec<Constant>,
    pub md5sum_first: String,
    pub md5sum_second: String,
    pub description: String,
    pub is_fixed_length: bool,
}

impl From<&MessageFile> for MessageSpecification {
    fn from(value: &MessageFile) -> Self {
        let md5sum = value.get_md5sum();
        let mut spec = Self {
            short_name: value.get_short_name(),
            package: value.get_package_name(),
            fields: value.get_fields().iter().map(Field::from).collect(),
            constants: value.get_constants().iter().map(Constant::from).collect(),
            md5sum_first: String::from(&md5sum[..md5sum.len() / 2]),
            md5sum_second: String::from(&md5sum[(md5sum.len() / 2)..]),
            description: String::from("Not a real description"),
            is_fixed_length: value.is_fixed_length(),
        };
        spec.fields.iter_mut().for_each(|field| {
            if field.package.is_none() {
                field.package = Some(value.get_package_name());
            }
        });
        spec
    }
}

#[derive(Deserialize, Serialize, Clone, Debug)]
pub struct ServiceSpecification {
    pub short_name: String,
    pub package: String,
    pub request_name: String,
    pub response_name: String,
    pub md5sum: String,
}

impl From<&ServiceFile> for ServiceSpecification {
    fn from(value: &ServiceFile) -> Self {
        Self {
            short_name: value.get_short_name(),
            package: value.get_package_name(),
            request_name: value.request().get_short_name(),
            response_name: value.response().get_short_name(),
            md5sum: value.get_md5sum(),
        }
    }
}
