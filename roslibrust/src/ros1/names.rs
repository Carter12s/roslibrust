lazy_static::lazy_static! {
    static ref GRAPH_NAME_REGEX: regex::Regex = regex::Regex::new(r"^([/~a-zA-Z]){1}([a-zA-Z0-9_/])*([A-z0-9_])$").unwrap();
}

#[derive(Clone, Debug, PartialEq)]
pub struct Name {
    inner: String,
}

impl Name {
    pub fn new(name: impl Into<String>) -> Option<Self> {
        let name: String = name.into();
        if is_valid(&name) {
            Some(Self { inner: name })
        } else {
            None
        }
    }

    pub fn resolve_to_global(&self, node_name: &Name) -> Self {
        if self.inner.starts_with('/') {
            self.clone()
        } else if self.inner.starts_with('~') {
            Name {
                inner: format!("{}/{}", node_name.inner, &self.inner[1..]),
            }
        } else {
            let components = node_name.inner.split("/").collect::<Vec<_>>();
            match components.len() {
                0..=1 => unreachable!("Node name {} must have at least one /", node_name.inner),
                2 => Name {
                    inner: format!("/{}", self.inner),
                },
                len => Name {
                    inner: format!(
                        "{}/{}",
                        components[1..len - 1].into_iter().fold(
                            String::new(),
                            |mut name, component| {
                                name.push('/');
                                name.push_str(component);
                                name
                            },
                        ),
                        self.inner
                    ),
                },
            }
        }
    }
}

fn is_valid(name: &str) -> bool {
    GRAPH_NAME_REGEX.is_match(name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_name_valid() {
        assert!(is_valid("base"));
        assert!(is_valid("relative/name"));
        assert!(is_valid("/global/name"));
        assert!(is_valid("~private/name"));

        // These are invalid names
        assert!(!is_valid("~"));
        assert!(!is_valid("~~"));
        assert!(!is_valid("_leading"));
    }

    // Examples pulled from http://wiki.ros.org/Names
    #[test]
    fn resolve_name() {
        let node1 = Name::new("/node1").unwrap();
        assert_eq!(
            Name::new("bar").unwrap().resolve_to_global(&node1),
            Name::new("/bar").unwrap()
        );
        assert_eq!(
            Name::new("/bar").unwrap().resolve_to_global(&node1),
            Name::new("/bar").unwrap()
        );
        assert_eq!(
            Name::new("~bar").unwrap().resolve_to_global(&node1),
            Name::new("/node1/bar").unwrap()
        );

        let node2 = Name::new("/wg/node2").unwrap();
        assert_eq!(
            Name::new("bar").unwrap().resolve_to_global(&node2),
            Name::new("/wg/bar").unwrap()
        );
        assert_eq!(
            Name::new("/bar").unwrap().resolve_to_global(&node2),
            Name::new("/bar").unwrap()
        );
        assert_eq!(
            Name::new("~bar").unwrap().resolve_to_global(&node2),
            Name::new("/wg/node2/bar").unwrap()
        );

        let node3 = Name::new("/wg/node3").unwrap();
        assert_eq!(
            Name::new("foo/bar").unwrap().resolve_to_global(&node3),
            Name::new("/wg/foo/bar").unwrap()
        );
        assert_eq!(
            Name::new("/foo/bar").unwrap().resolve_to_global(&node3),
            Name::new("/foo/bar").unwrap()
        );
        assert_eq!(
            Name::new("~foo/bar").unwrap().resolve_to_global(&node3),
            Name::new("/wg/node3/foo/bar").unwrap()
        );
    }
}
