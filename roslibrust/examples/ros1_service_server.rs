roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

#[cfg(feature = "ros1")]
#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use log::*;
    use roslibrust::ros1::NodeHandle;
    use std::sync::{Arc, Mutex};

    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps()
        .init()
        .unwrap();

    let nh = NodeHandle::new("http://localhost:11311", "service_server_rs").await?;
    log::info!("Connected!");

    // Because our service server can run from any thread at any time
    // and *multiple service requests can process in parallel*
    // the service function is not allowed to hold any references to external data
    // all data must be owned by the service function
    // See: https://tokio.rs/tokio/tutorial/spawning
    // To work around this we want to move a "protected" version of our state into the function
    let bool_state = Arc::new(Mutex::new(false));

    let bool_state_copy = bool_state.clone(); // Make a copy (of the Arc, not the contents!)
    let server_fn = move |request: std_srvs::SetBoolRequest| {
        log::info!("Got request to set bool: {request:?}");
        // Actually set the bool
        *bool_state_copy.lock().unwrap() = request.data;
        Ok(std_srvs::SetBoolResponse {
            success: true,
            message: "You set my bool!".to_string(),
        })
    };

    // Start our service running!
    let _handle = nh
        .advertise_service::<std_srvs::SetBool, _>("~/my_set_bool", server_fn)
        .await?;
    info!("Service has started");

    // Setup a task to kill this process when ctrl_c comes in:
    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        std::process::exit(0);
    });

    // As long as _handle is kept alive our service will continue to run

    // For funsies we can also spawn a task to periodically call our service
    let service_client = nh
        .service_client::<std_srvs::SetBool>("~/my_set_bool")
        .await?;
    tokio::spawn(async move {
        let mut bool = false;
        loop {
            bool = !bool;
            service_client
                .call(&std_srvs::SetBoolRequest { data: bool })
                .await
                .unwrap();
            tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
        }
    });

    // We can still access our shared state, we just have to do it safely
    loop {
        let cur_bool = *bool_state.lock().unwrap();
        info!("Current value of our bool: {cur_bool}");
        tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
    }
}

#[cfg(not(feature = "ros1"))]
fn main() {
    eprintln!("This example does nothing without compiling with the feature 'ros1'");
}
