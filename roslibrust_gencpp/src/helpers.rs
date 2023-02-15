use minijinja::value::Value;

use crate::spec::MessageSpecification;

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
