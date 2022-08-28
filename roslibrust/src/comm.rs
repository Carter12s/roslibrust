use std::{fmt::Display, str::FromStr, string::ToString};

use anyhow::bail;
use async_trait::async_trait;
use futures_util::SinkExt;
use log::debug;
use serde_json::json;
use tokio_tungstenite::tungstenite::Message;

use crate::{RosLibRustResult, RosMessageType, Writer};

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
            // Leaving other unimplmented to catch bugs
            // TODO implement these
            _ => bail!("Un-recognized op: {}", s),
        })
    }
}

/// Describes the low level comm capabilities of talking to a rosbridge server
#[async_trait]
pub(crate) trait RosBridgeComm {
    async fn subscribe(&mut self, topic: &str, msg_type: &str) -> RosLibRustResult<()>;
    async fn unsubscribe(&mut self, topic: &str) -> RosLibRustResult<()>;
    async fn publish<T: RosMessageType>(&mut self, topic: &str, msg: T) -> RosLibRustResult<()>;
    // TODO may provide a verison of advertise that takes explicit string for msg_type
    async fn advertise<T: RosMessageType>(&mut self, topic: &str) -> RosLibRustResult<()>;
    async fn call_service<Req: RosMessageType>(
        &mut self,
        service: &str,
        id: &str,
        req: Req,
    ) -> RosLibRustResult<()>;
    async fn unadvertise(&mut self, topic: &str) -> RosLibRustResult<()>;
}

#[async_trait]
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
        self.send(msg).await?;
        Ok(())
    }

    async fn publish<T: RosMessageType>(&mut self, topic: &str, msg: T) -> RosLibRustResult<()> {
        let msg = json!(
            {
                "op": Ops::Publish.to_string(),
                "topic": topic,
                "type": T::ROS_TYPE_NAME,
                "msg": &msg,
            }
        );
        let msg = Message::Text(msg.to_string());
        self.send(msg).await?;
        Ok(())
    }

    async fn advertise<T: RosMessageType>(&mut self, topic: &str) -> RosLibRustResult<()> {
        let msg = json!(
            {
                "op": Ops::Advertise.to_string(),
                "topic": topic.to_string(),
                "type": T::ROS_TYPE_NAME,
            }
        );
        let msg = Message::Text(msg.to_string());
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
                "args": [req],
            }
        );
        let msg = Message::Text(msg.to_string());
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
        self.send(msg).await?;
        Ok(())
    }
}
