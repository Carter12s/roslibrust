#[test]
fn test_crawl() {
    let packages = roslibrust_codegen::utils::crawl(vec![env!("CARGO_MANIFEST_DIR").into()]);
    assert_eq!(packages.len(), 1);
}
