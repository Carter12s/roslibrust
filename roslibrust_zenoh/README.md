# RosLibRust Zenoh

This crate provides a Zenoh client that is compatible with the zenoh-ros1-plugin / zenoh-ros1-bridge.

The plugin / bridge performs "topic mangling" that makes it challenging to directly subscribe to the bridged topics from zenoh.

The goal of this crate is to provide an effective intermediary between ros1 and zenoh, and eventually unify this behind the single TopicProvider trait.