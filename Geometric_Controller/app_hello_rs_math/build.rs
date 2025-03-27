use std::env;
use std::path::PathBuf;

fn main() {
    // 设置目标头文件路径
    let header_path = "include/stabilizer_types.h";

    // 输出绑定文件的路径
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap()).join("bindings.rs");

    // 调用 bindgen
    bindgen::Builder::default()
        .header(header_path) // 指定头文件
        .layout_tests(false)
        .generate_comments(true) // 是否保留注释
        .generate() // 生成绑定
        .expect("Unable to generate bindings") // 错误处理
        .write_to_file(out_path) // 写入到输出路径
        .expect("Couldn't write bindings!");
}

