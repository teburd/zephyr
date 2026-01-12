use logos::Logos;
pub use logos::Span;
pub mod lex;
pub mod parse;

pub type Error = (String, Span);
pub type Result<T> = std::result::Result<T, Error>;
pub use parse::{Dts, DeviceNode, PropertyValue, Cell};

pub fn parse_str<'source>(src: &'source str) -> Result<Vec<Dts<'source>>> {
    let mut lexer = lex::Token::lexer(src);

    parse::parse_root(&mut lexer)
}
