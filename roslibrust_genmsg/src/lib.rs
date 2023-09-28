use cpp::ROS_TYPE_TO_CPP_TYPE_MAP;
use minijinja::{context, Template};
use roslibrust_codegen::{MessageFile, ServiceFile};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

use roslibrust_codegen::utils::Package;

mod cpp;
mod helpers;
mod spec;

use spec::{MessageSpecification, ServiceSpecification};

#[derive(Clone, Debug)]
pub struct IncludedNamespace {
    /// The package namespace being included
    pub package: String,
    /// The path to the package
    pub path: PathBuf,
}

impl From<Package> for IncludedNamespace {
    fn from(value: Package) -> Self {
        IncludedNamespace {
            package: value.name,
            path: value.path,
        }
    }
}

impl From<&Package> for IncludedNamespace {
    fn from(value: &Package) -> Self {
        IncludedNamespace {
            package: value.name.clone(),
            path: value.path.clone(),
        }
    }
}

pub struct MessageGenOutput {
    /// The short name of the ROS message
    pub message_name: String,
    /// The package namespace
    pub package_name: String,
    /// The generated source for the message
    pub message_source: String,
}

pub struct ServiceGenOutput {
    /// The short name of the ROS service
    pub service_name: String,
    /// The package namespace
    pub package_name: String,
    /// The generated source for the request structure
    pub request_source: String,
    /// The generated source for the response structure
    pub response_source: String,
    /// The generated source for the service structure
    pub service_source: String,
}

type Filter = dyn Fn(minijinja::value::Value) -> minijinja::value::Value + Send + Sync;

/// A builder to customize the generation of code from ROS message and service
/// definitions.
pub struct CodeGeneratorBuilder<'a> {
    msg_paths: Vec<PathBuf>,
    msg_template: &'a str,
    srv_template: Option<&'a str>,
    typename_conversion_mapping: Option<HashMap<String, String>>,
    filters: Vec<(String, Box<Filter>)>,
}

impl<'a> CodeGeneratorBuilder<'a> {
    /// Create the builder with the minimum required dependencies of code generation
    pub fn new<P: AsRef<Path>>(search_paths: &[P], msg_template: &'a str) -> Self {
        Self {
            msg_paths: search_paths.iter().map(|p| p.as_ref().to_owned()).collect(),
            msg_template,
            srv_template: None,
            typename_conversion_mapping: None,
            filters: vec![],
        }
    }

    /// Performs discovery of ROS messages, services, and actions, resolves their
    /// dependency graph and builds a `CodeGenerator`.
    pub fn build(self) -> std::io::Result<CodeGenerator<'a>> {
        let (messages, services) =
            roslibrust_codegen::find_and_parse_ros_messages(&self.msg_paths)?;
        let (messages, services) =
            roslibrust_codegen::resolve_dependency_graph(messages, services).unwrap();

        let mut env = helpers::prepare_environment(
            self.msg_template,
            self.srv_template.unwrap_or(""),
            self.typename_conversion_mapping.clone(),
        );

        self.filters
            .into_iter()
            .for_each(|(name, filter)| env.add_filter(name, filter));

        Ok(CodeGenerator {
            messages,
            services,
            template_environment: env,
        })
    }

    /// Add a service template for generating service sources.
    pub fn service_template(mut self, srv_template: &'a str) -> Self {
        self.srv_template = Some(srv_template);
        self
    }

    /// Add a map of type names from ROS types to native types using the filter
    /// `typename_conversion`.
    pub fn add_type_mapping(mut self, map: HashMap<String, String>) -> Self {
        self.typename_conversion_mapping = Some(map);
        self
    }

    /// Add a custom filter which can be used by the message and service templates.
    pub fn add_filter<F>(mut self, name: &str, filter: F) -> Self
    where
        F: Fn(minijinja::value::Value) -> minijinja::value::Value + Send + Sync + 'static,
    {
        self.filters.push((name.to_owned(), Box::new(filter)));
        self
    }
}

pub struct CodeGenerator<'a> {
    messages: Vec<MessageFile>,
    services: Vec<ServiceFile>,
    template_environment: minijinja::Environment<'a>,
}

impl<'a> CodeGenerator<'a> {
    /// Generate source for all ROS messages (and actions)
    pub fn generate_messages(&self) -> Result<Vec<MessageGenOutput>, minijinja::Error> {
        self.messages
            .iter()
            .map(|msg| {
                let message_source = fill_message_template(
                    &self.template_environment.get_template("message").unwrap(),
                    msg,
                )?;
                Ok(MessageGenOutput {
                    message_name: msg.get_short_name(),
                    package_name: msg.get_package_name(),
                    message_source,
                })
            })
            .collect::<Result<Vec<_>, _>>()
    }

    /// Generate source for all ROS services
    pub fn generate_services(&self) -> Result<Vec<ServiceGenOutput>, minijinja::Error> {
        self.services
            .iter()
            .map(|srv| {
                let request_source = fill_message_template(
                    &self.template_environment.get_template("message").unwrap(),
                    srv.request(),
                )?;
                let response_source = fill_message_template(
                    &self.template_environment.get_template("message").unwrap(),
                    srv.response(),
                )?;
                let service_source = fill_service_template(
                    &self.template_environment.get_template("service").unwrap(),
                    srv,
                )?;
                Ok(ServiceGenOutput {
                    service_name: srv.get_short_name(),
                    package_name: srv.get_package_name(),
                    request_source,
                    response_source,
                    service_source,
                })
            })
            .collect::<Result<Vec<_>, _>>()
    }
}

/// Create a code generator for C++ headers.
pub fn make_cpp_generator<P: AsRef<Path>>(search_paths: &[P]) -> std::io::Result<CodeGenerator> {
    CodeGeneratorBuilder::new(search_paths, cpp::MESSAGE_HEADER_TMPL)
        .add_type_mapping(ROS_TYPE_TO_CPP_TYPE_MAP.clone())
        .service_template(cpp::SERVICE_HEADER_TMPL)
        .build()
}

fn fill_message_template(
    template: &Template,
    msg_data: &MessageFile,
) -> Result<String, minijinja::Error> {
    let context = context! {
        spec => MessageSpecification::from(msg_data),
    };
    template.render(&context)
}

fn fill_service_template(
    template: &Template,
    srv_data: &ServiceFile,
) -> Result<String, minijinja::Error> {
    let context = context! {
        spec => ServiceSpecification::from(srv_data),
    };
    template.render(&context)
}
