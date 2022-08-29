string node_name
int64 pid

# Node is created, but is not yet initialized.
uint8 STATUS_UNINITIALIZED=0
# Node is initialized, but not connected.
uint8 STATUS_DISCONNECTED=1
# Node is initialized, connected, and running successfully.
uint8 STATUS_RUNNING=2
# Node is initialized and connected, but has a run error.
uint8 STATUS_RUN_ERROR=3
# Node was running, and is now shutting down.
uint8 STATUS_SHUTTING_DOWN=4
# Node is stopped.
uint8 STATUS_SHUTDOWN=5
uint8 status

string TEST_STR=/topic