use roslibrust_codegen::RosServiceType;
use std::{future::Future, marker::PhantomData};
use tokio::sync::{mpsc, oneshot};

pub struct ServiceCall {
    pub request: Vec<u8>,
    pub responder: oneshot::Sender<Vec<u8>>,
}

pub struct ServiceServer<T> {
    service_name: String,
    receiver: mpsc::Receiver<ServiceCall>,
    phantom: PhantomData<T>,
}

impl<T: RosServiceType> ServiceServer<T> {
    pub fn new(service_name: &str, receiver: mpsc::Receiver<ServiceCall>) -> Self {
        Self {
            service_name: service_name.to_owned(),
            receiver,
            phantom: PhantomData,
        }
    }

    pub async fn serve_once<F, Fut>(&mut self, handler: F) -> Result<(), Box<dyn std::error::Error>>
    where
        F: FnOnce(T::Request) -> Fut,
        Fut: Future<Output = T::Response>,
    {
        match self.receiver.recv().await {
            Some(call) => {
                let request = serde_rosmsg::from_slice::<T::Request>(call.request.as_slice())?;
                let response = handler(request).await;
                let _ = call
                    .responder
                    .send(serde_rosmsg::to_vec(&response).unwrap());
                Ok(())
            }
            None => {
                log::error!(
                    "Service server receiver channel disconnected for service {}",
                    self.service_name
                );
                Err(Box::new(std::io::Error::from(
                    std::io::ErrorKind::ConnectionAborted,
                )))
            }
        }
    }
}
