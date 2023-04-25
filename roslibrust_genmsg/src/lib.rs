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
    pub message_name: String,
    pub package_name: String,
    pub message_source: String,
}

pub struct ServiceGenOutput {
    pub service_name: String,
    pub package_name: String,
    pub request_source: String,
    pub response_source: String,
    pub service_source: String,
}

pub struct CodeGeneratorBuilder<'a> {
    msg_paths: Vec<PathBuf>,
    msg_template: &'a str,
    srv_template: Option<&'a str>,
    typename_conversion_mapping: HashMap<String, String>,
}

impl<'a> CodeGeneratorBuilder<'a> {
    pub fn new<P: AsRef<Path>>(
        search_paths: &[P],
        msg_template: &'a str,
        typename_conversion_mapping: HashMap<String, String>,
    ) -> Self {
        Self {
            msg_paths: search_paths.iter().map(|p| p.as_ref().to_owned()).collect(),
            msg_template,
            srv_template: None,
            typename_conversion_mapping,
        }
    }

    pub fn build(self) -> std::io::Result<CodeGenerator<'a>> {
        let (messages, services) = roslibrust_codegen::find_and_parse_ros_messages(self.msg_paths)?;
        let (messages, services) =
            roslibrust_codegen::resolve_dependency_graph(messages, services).unwrap();

        let env = helpers::prepare_environment(
            self.msg_template,
            self.srv_template.unwrap_or(""),
            self.typename_conversion_mapping.clone(),
        );

        Ok(CodeGenerator {
            messages,
            services,
            template_environment: env,
        })
    }

    pub fn service_template(mut self, srv_template: &'a str) -> Self {
        self.srv_template = Some(srv_template);
        self
    }
}

pub struct CodeGenerator<'a> {
    messages: Vec<MessageFile>,
    services: Vec<ServiceFile>,
    template_environment: minijinja::Environment<'a>,
}

impl<'a> CodeGenerator<'a> {
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

pub fn make_cpp_generator<P: AsRef<Path>>(search_paths: &[P]) -> std::io::Result<CodeGenerator> {
    CodeGeneratorBuilder::new(
        search_paths,
        cpp::MESSAGE_HEADER_TMPL,
        ROS_TYPE_TO_CPP_TYPE_MAP.clone(),
    )
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
