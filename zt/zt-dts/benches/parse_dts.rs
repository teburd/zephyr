use criterion::{Criterion, criterion_group, criterion_main};
use logos::Logos;
use std::hint::black_box;
use zt_dts::{lex::Token, parse::parse_root};

fn parse_dts_benchmark(c: &mut Criterion) {
    c.bench_function("parse test.dts", |b| {
        let input = include_str!("../dts/test.dts");
        b.iter(|| {
            let mut lexer = Token::lexer(input);
            black_box(parse_root(&mut lexer).unwrap());
        });
    });
}

criterion_group!(benches, parse_dts_benchmark);
criterion_main!(benches);
