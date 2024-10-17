use criterion::{criterion_group, criterion_main, Criterion};
use std::{
    hint::black_box,
    sync::{Arc, Mutex},
};

struct BenchContext {
    publisher: roslibrust::ros1::Publisher<roslibrust_test::ros1::sensor_msgs::Image>,
    subscriber: roslibrust::ros1::Subscriber<roslibrust_test::ros1::sensor_msgs::Image>,
    image: roslibrust_test::ros1::sensor_msgs::Image,
    // Need to keep alive
    _client: roslibrust::ros1::NodeHandle,
}

async fn setup_bench_context() -> BenchContext {
    let client = roslibrust::ros1::NodeHandle::new("http://localhost:11311", "image_bench_rs")
        .await
        .unwrap();
    let publisher = client
        .advertise::<roslibrust_test::ros1::sensor_msgs::Image>("/image_bench", 1, false)
        .await
        .unwrap();
    let subscriber = client
        .subscribe::<roslibrust_test::ros1::sensor_msgs::Image>("/image_bench", 1)
        .await
        .unwrap();

    // Wait for pub / sub to establish connection
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

    let image = roslibrust_test::ros1::sensor_msgs::Image {
        header: Default::default(),
        height: 100,
        width: 100,
        encoding: "rgb8".to_owned(),
        is_bigendian: 0,
        step: 300,
        data: vec![0; 1920 * 1080 * 3],
    };

    BenchContext {
        publisher,
        subscriber,
        image,
        _client: client,
    }
}

async fn bench_iteration(context: &mut BenchContext) {
    context.publisher.publish(&context.image).await.unwrap();
    let received_image = context.subscriber.next().await.unwrap().unwrap();
    black_box(received_image);
}

fn criterion_benchmark(c: &mut Criterion) {
    env_logger::init();

    // Create a tokio runtime so we can be async
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap();

    let context = runtime.block_on(async {
        let context = setup_bench_context().await;
        Arc::new(Mutex::new(context))
    });

    c.bench_function("image_bench", |b| {
        b.to_async(&runtime).iter(|| async {
            let mut context = context.lock().unwrap();
            bench_iteration(&mut context).await
        })
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
