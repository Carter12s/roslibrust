use super::{names::Name, NodeHandle};

// TODO: someday I'd like to define a trait alias here for a ServerFunction
// Currently unstable:
// https://doc.rust-lang.org/beta/unstable-book/language-features/trait-alias.html
// trait ServerFunction<T> = Fn(T::Request) -> Err(T::Response, Box<dyn std::error::Error + Send + Sync>) + Send + Sync + 'static;

/// ServiceServer is simply a lifetime control
/// The underlying ServiceServer is kept alive while object is kept alive.
/// Dropping this object, un-advertises the underlying service with rosmaster
// TODO: is this actually the API we want to provide?
// Maybe we should just let people manually call an unadvertise_service method?
pub struct ServiceServer {
    service_name: Name,
    node_handle: NodeHandle,
}

impl ServiceServer {
    pub fn new(service_name: Name, node_handle: NodeHandle) -> Self {
        Self {
            service_name,
            node_handle,
        }
    }
}

impl Drop for ServiceServer {
    fn drop(&mut self) {
        unimplemented!()
        // Need to eventually define and call this:
        // self.node_handle
        //     .unadvertise_service(&self.service_name.to_string());
    }
}

/// Internal type held by the NodeServer to keep track of a given service server
pub(crate) struct ServiceServerLink {
    service_name: String,
    // Internal handle to the type erased version of the server function passed in to advertise_service
    method:
        Box<dyn Fn(Vec<u8>) -> Result<(), Box<dyn std::error::Error + Send + Sync>> + Send + Sync>,
}
