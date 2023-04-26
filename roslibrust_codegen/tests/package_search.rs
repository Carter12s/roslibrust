#[test]
fn test_crawl() {
    let packages = roslibrust_codegen::utils::crawl(&[env!("CARGO_MANIFEST_DIR")]);
    assert_eq!(packages.len(), 1);
}
