use crate::{RosLibRustResult, Writer};
use anyhow::bail;
use futures_util::SinkExt;
use log::debug;
use roslibrust_common::RosMessageType;
use serde_json::json;
use std::{fmt::Display, str::FromStr, string::ToString};
use tokio_tungstenite::tungstenite::Message;

/// Describes all documented rosbridge server operations
pub(crate) enum Ops {
    // These are in the definition, but not used right now
    #[allow(dead_code)]
    Status,
    #[allow(dead_code)]
    SetLevel,
    #[allow(dead_code)]
    Fragment,
    #[allow(dead_code)]
    Auth,
    // Below here are in use
    Advertise,
    Unadvertise,
    Publish,
    Subscribe,
    Unsubscribe,
    CallService,
    ServiceResponse,
    AdvertiseService,
    UnadvertiseService,
}

impl Display for Ops {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s: &str = self.into();
        write!(f, "{}", s)
    }
}

// TODO should replace this with impl Serialize
impl Into<&str> for &Ops {
    fn into(self) -> &'static str {
        match self {
            // These are unimplemented features of the library right now
            // Leaving them unimplemented here to try to catch bugs
            // TODO implement these
            Ops::Status => unimplemented!(),
            Ops::SetLevel => unimplemented!(),
            Ops::Fragment => unimplemented!(),
            Ops::Auth => unimplemented!(),
            Ops::Advertise => "advertise",
            Ops::Unadvertise => "unadvertise",
            Ops::Publish => "publish",
            Ops::Subscribe => "subscribe",
            Ops::Unsubscribe => "unsubscribe",
            Ops::CallService => "call_service",
            Ops::ServiceResponse => "service_response",
            Ops::AdvertiseService => "advertise_service",
            Ops::UnadvertiseService => "unadvertise_service",
        }
    }
}

// TODO should replace this with deserialize
impl FromStr for Ops {
    type Err = anyhow::Error;
    fn from_str(s: &str) -> Result<Self, anyhow::Error> {
        Ok(match s {
            "advertise" => Ops::Advertise,
            "unadvertise" => Ops::Unadvertise,
            "publish" => Ops::Publish,
            "subscribe" => Ops::Subscribe,
            "unsubscribe" => Ops::Unsubscribe,
            "call_service" => Ops::CallService,
            "service_response" => Ops::ServiceResponse,
            "advertise_service" => Ops::AdvertiseService,
            "unadvertise_service" => Ops::UnadvertiseService,
            // Leaving other unimplemented to catch bugs
            // TODO implement these
            _ => bail!("Un-recognized op: {}", s),
        })
    }
}

/// Describes the low level comm capabilities of talking to a rosbridge server
/// This trait exists because we haven't wrapped Writer in our own type
/// So we're defining this trait on a foreign type, since we didn't end up
/// using this trait for mocking. I'm inclined to replace it, and move the
/// impls directly into some wrapper around [Writer]
pub(crate) trait RosBridgeComm {
    async fn subscribe(&mut self, topic: &str, msg_type: &str) -> RosLibRustResult<()>;
    async fn unsubscribe(&mut self, topic: &str) -> RosLibRustResult<()>;
    async fn publish<T: RosMessageType>(&mut self, topic: &str, msg: &T) -> RosLibRustResult<()>;
    async fn advertise<T: RosMessageType>(&mut self, topic: &str) -> RosLibRustResult<()>;
    async fn advertise_str(&mut self, topic: &str, msg_type: &str) -> RosLibRustResult<()>;
    async fn call_service<Req: RosMessageType>(
        &mut self,
        service: &str,
        id: &str,
        req: Req,
    ) -> RosLibRustResult<()>;
    async fn unadvertise(&mut self, topic: &str) -> RosLibRustResult<()>;
    async fn advertise_service(&mut self, topic: &str, srv_type: &str) -> RosLibRustResult<()>;
    async fn unadvertise_service(&mut self, topic: &str) -> RosLibRustResult<()>;
    async fn service_response(
        &mut self,
        topic: &str,
        id: Option<String>,
        is_success: bool,
        response: serde_json::Value,
    ) -> RosLibRustResult<()>;
}

impl RosBridgeComm for Writer {
    async fn subscribe(&mut self, topic: &str, msg_type: &str) -> RosLibRustResult<()> {
        let msg = json!(
        {
        "op": Ops::Subscribe.to_string(),
        "topic": topic,
        "type": msg_type,
        }
        );
        let msg = Message::Text(msg.to_string());
        debug!("Sending subscribe: {:?}", &msg);
        self.send(msg).await?;
        Ok(())
    }

    async fn unsubscribe(&mut self, topic: &str) -> RosLibRustResult<()> {
        let msg = json!(
        {
        "op": Ops::Unsubscribe.to_string(),
        "topic": topic,
        }
        );
        let msg = Message::Text(msg.to_string());
        debug!("Sending unsubscribe: {:?}", &msg);
        self.send(msg).await?;
        Ok(())
    }

    async fn publish<T: RosMessageType>(&mut self, topic: &str, msg: &T) -> RosLibRustResult<()> {
        let msg = json!(
            {
                "op": Ops::Publish.to_string(),
                "topic": topic,
                "type": T::ROS_TYPE_NAME,
                "msg": &msg,
            }
        );
        let msg = Message::Text(msg.to_string());
        debug!("Sending publish: {:?}", &msg);
        self.send(msg).await?;
        Ok(())
    }

    async fn advertise<T: RosMessageType>(&mut self, topic: &str) -> RosLibRustResult<()> {
        self.advertise_str(topic, T::ROS_TYPE_NAME).await
    }

    // Identical to advertise, but allows providing a string argument for the topic type
    // This is important as the type is erased in our list of publishers, and not available
    // when we try to reconnect
    async fn advertise_str(&mut self, topic: &str, topic_type: &str) -> RosLibRustResult<()> {
        let msg = json!(
            {
                "op": Ops::Advertise.to_string(),
                "topic": topic.to_string(),
                "type": topic_type,
            }
        );
        let msg = Message::Text(msg.to_string());
        debug!("Sending advertise: {:?}", &msg);
        self.send(msg).await?;
        Ok(())
    }

    async fn call_service<Req: RosMessageType>(
        &mut self,
        service: &str,
        id: &str,
        req: Req,
    ) -> RosLibRustResult<()> {
        let msg = json!(
            {
                "op": Ops::CallService.to_string(),
                "service": service,
                "id": id,
                "args": req,
            }
        );
        let msg = Message::Text(msg.to_string());
        debug!("Sending call_service: {:?}", &msg);
        self.send(msg).await?;
        Ok(())
    }

    async fn unadvertise(&mut self, topic: &str) -> RosLibRustResult<()> {
        debug!("Sending unadvertise on {}", topic);
        let msg = json! {
            {
                "op": Ops::Unadvertise.to_string(),
                "topic": topic
            }
        };
        let msg = Message::Text(msg.to_string());
        debug!("Sending unadvertise: {:?}", &msg);
        self.send(msg).await?;
        Ok(())
    }

    async fn advertise_service(&mut self, srv_name: &str, srv_type: &str) -> RosLibRustResult<()> {
        debug!("Sending advertise service on {} w/ {}", srv_name, srv_type);
        let msg = json! {
            {
                "op": Ops::AdvertiseService.to_string(),
                "type": srv_type,
                "service": srv_name
            }
        };
        let msg = Message::Text(msg.to_string());
        self.send(msg).await?;
        Ok(())
    }

    async fn unadvertise_service(&mut self, topic: &str) -> RosLibRustResult<()> {
        debug!("Sending unadvertise service on {topic}");
        let msg = json! {
            {
                "op": Ops::UnadvertiseService.to_string(),
                "service": &topic
            }
        };
        let msg = Message::Text(msg.to_string());
        self.send(msg).await?;
        Ok(())
    }

    async fn service_response(
        &mut self,
        topic: &str,
        id: Option<String>,
        is_success: bool,
        response: serde_json::Value,
    ) -> RosLibRustResult<()> {
        debug!(
            "Sending service response on {:?} with {:?}, {:?}, {:?}",
            topic, id, is_success, response
        );
        let msg = json! {
            {
                "op": Ops::ServiceResponse.to_string(),
                "service": topic,
                "id": id,
                "result": is_success,
                "values": response,
            }
        };
        let msg = Message::Text(msg.to_string());
        debug!("Sending service_response: {:?}", &msg);
        self.send(msg).await?;
        Ok(())
    }
}
