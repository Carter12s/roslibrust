//! Not actually an example for this crate, just peeping on how discovery for zenoh-ros1-bridge works

#[tokio::main]
async fn main() {
    let session = zenoh::open(zenoh::Config::default()).await.unwrap();

    let sub = session
        .declare_subscriber("ros1_discovery_info/**")
        .await
        .unwrap();

    loop {
        let sample = sub.recv_async().await.unwrap();
        println!("Got sample: {sample:?}");
    }
}
