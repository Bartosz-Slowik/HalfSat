fn main() {
    println!("cargo:rerun-if-changed=native/ssdv_wrap.c");
    println!("cargo:rerun-if-changed=../lib/ssdv/ssdv.c");
    println!("cargo:rerun-if-changed=../lib/ssdv/rs8.c");
    println!("cargo:rerun-if-changed=../lib/ssdv/ssdv.h");
    println!("cargo:rerun-if-changed=../lib/ssdv/rs8.h");

    cc::Build::new()
        .file("native/ssdv_wrap.c")
        .file("../lib/ssdv/ssdv.c")
        .file("../lib/ssdv/rs8.c")
        .include("../lib/ssdv")
        .warnings(false)
        .compile("ssdv_native");
}
