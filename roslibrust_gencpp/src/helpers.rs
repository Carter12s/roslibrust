use crate::spec::{Field, MessageSpecification, ROS_TYPE_TO_CPP_TYPE_MAP};
use minijinja::value::Value;

/// Given a message specification determines if the message has a header
pub fn has_header(value: Value) -> bool {
    if let Ok(value) = serde_json::to_value(value) {
        if let Ok(spec) = serde_json::from_value::<MessageSpecification>(value) {
            spec.fields
                .iter()
                .any(|field| field.field_type.as_str() == "Header")
        } else {
            false
        }
    } else {
        false
    }
}

/// Given a message specification determines if the message is a fixed length
pub fn is_fixed_length(value: Value) -> bool {
    if let Ok(value) = serde_json::to_value(value) {
        if let Ok(spec) = serde_json::from_value::<MessageSpecification>(value) {
            spec.is_fixed_length
        } else {
            false
        }
    } else {
        false
    }
}

/// Takes a ROS type as a string and converts it to a C++ type as a string
pub fn cpp_type(value: Value) -> Value {
    let value = serde_json::to_value(value).unwrap();
    let ros_type = value.as_str().unwrap();
    Value::from_serializable(ROS_TYPE_TO_CPP_TYPE_MAP.get(ros_type).unwrap())
}

pub fn cpp_literal(value: Value) -> Value {
    value
}

pub fn is_intrinsic_type(value: Value) -> bool {
    if let Ok(value) = serde_json::to_value(value) {
        if let Ok(field) = serde_json::from_value::<Field>(value) {
            field.is_intrinsic_type()
        } else {
            false
        }
    } else {
        false
    }
}
