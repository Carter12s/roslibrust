use crate::spec::{Field, MessageSpecification};
use minijinja::{value::Value, Environment};
use std::collections::HashMap;

pub fn prepare_environment<'a>(
    message_template: &'a str,
    service_template: &'a str,
    typename_conversion_mapping: Option<HashMap<String, String>>,
) -> Environment<'a> {
    let mut env = Environment::new();
    env.add_function("has_header", has_header);
    env.add_function("is_fixed_length", is_fixed_length);
    env.add_function("is_intrinsic_type", is_intrinsic_type);
    env.add_function("is_array", is_array_type);
    env.add_function("is_fixed_array", is_fixed_size_array_type);
    env.add_filter("fixed_size_array_size", fixed_size_array_size);
    if let Some(map) = typename_conversion_mapping {
        env.add_filter("typename_conversion", move |v: Value| {
            let value = serde_json::to_value(v).unwrap();
            let ros_type = value.as_str().unwrap();
            let typename = match map.get(ros_type) {
                Some(native_type) => native_type.clone(),
                None => ros_type.to_string(),
            };
            Value::from_serializable(&typename)
        });
    }
    env.add_template("message", message_template).unwrap();
    env.add_template("service", service_template).unwrap();
    env
}

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

pub fn is_array_type(value: Value) -> bool {
    if let Ok(value) = serde_json::to_value(value) {
        if let Ok(field) = serde_json::from_value::<Field>(value) {
            field.is_array_type()
        } else {
            false
        }
    } else {
        false
    }
}

pub fn is_fixed_size_array_type(value: Value) -> bool {
    if let Ok(value) = serde_json::to_value(value) {
        if let Ok(field) = serde_json::from_value::<Field>(value) {
            field.is_fixed_size_array_type()
        } else {
            false
        }
    } else {
        false
    }
}

pub fn fixed_size_array_size(value: Value) -> Value {
    let fixed_size = if let Ok(value) = serde_json::to_value(value) {
        if let Ok(field) = serde_json::from_value::<Field>(value) {
            field.fixed_size_array_size()
        } else {
            0
        }
    } else {
        0
    };
    Value::from_serializable(&fixed_size)
}
