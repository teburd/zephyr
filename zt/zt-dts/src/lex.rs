use logos::{Lexer, Logos, Source, Span};

pub type Error = (String, Span);

pub type Result<T> = std::result::Result<T, Error>;

// Capture all source from /* to the first */ non-greedily
fn capture_comment<'source>(lex: &mut Lexer<'source, Token<'source>>) -> Option<&'source str> {
    let rem = lex.remainder();
    if let Some(close_pos) = rem.find("*/") {
        lex.bump(close_pos + 2);
        rem.slice(0..close_pos)
    } else {
        None
    }
}

fn capture_line_comment<'source>(lex: &mut Lexer<'source, Token<'source>>) -> &'source str {
    let rem = lex.remainder();
    let close_pos = rem.find('\n').unwrap_or(rem.len());
    lex.bump(close_pos + 1);
    rem.slice(0..close_pos).unwrap()
}

// Some common regex for DTS, noted here from DTC as the
// DTS spec if woefully limited in what it actually accepts
// Funnily... PROPNODECHAR isn't really correct as a node name
// may be an absolute or relative path. Oh well.
//const PROPNODECHAR: &str = "[a-zA-Z0-9,._+*#?@-]";
//const PATHCHAR: &str = "({PROPNODECHAR}|[/])";
//const LABEL: &str = "[a-zA-Z_][a-zA-Z0-9_]*";
//const STRING: &str = "\"[\\.]*\"";

/// All meaningful DTS tokens.
#[derive(Debug, Logos, PartialEq)]
#[logos(skip r"[\r\f]+")]
pub enum Token<'source> {
    #[regex("\n|\r\n")]
    NewLine,

    #[regex("[ \t]")]
    WhiteSpace,

    #[token("/*", capture_comment)]
    #[token("//", capture_line_comment)]
    Comment(&'source str),

    #[token("/dts-v1/")]
    DtsV1,

    #[regex("/include/[ \t]+\"[A-Za-z_.\\\\/]+\"")]
    Include(&'source str),

    #[regex("/delete-node/[ \t]+\"[A-Za-z_.\\\\/]+\"")]
    DeleteNode(&'source str),

    #[regex("[A-Za-z0-9_]+:")]
    Label(&'source str),

    #[token("false", |_| false)]
    #[token("true", |_| true)]
    Bool(bool),

    #[token("{")]
    CurlyOpen,

    #[token("}")]
    CurlyClose,

    #[token("[")]
    SquareOpen,

    #[token("]")]
    SquareClose,

    #[token("<")]
    AngleOpen,

    #[token(">")]
    AngleClose,

    #[token(":")]
    Colon,

    #[token(",")]
    Comma,

    #[token(";")]
    SemiColon,

    #[token("null")]
    Null,

    #[token("=")]
    Assign,

    #[token("+")]
    Plus,

    #[token("-")]
    Minus,

    #[token("*")]
    Multiply,

    #[token("[ \t]+/[ \t]+")]
    Slash,

    #[token("%")]
    Modulo,

    #[token("&")]
    Ampersand,

    #[token("|")]
    BitOr,

    #[token("^")]
    BitXor,

    #[token("~")]
    BitNot,

    #[token("<<")]
    ShiftLeft,

    #[token(">>")]
    ShiftRight,

    #[token("&&")]
    LogicalAnd,

    #[token("||")]
    LogicalOr,

    #[token("!")]
    LogicalNot,

    #[token("?")]
    Question,

    #[regex("&[{]?[A-Za-z/][A-Za-z/0-9,._+*#?@-]*[}]?")]
    PHandle(&'source str),

    //TODO dtc actually uses a stateful lexer here
    //noting that a PropNodeName always comes after a ; or {
    //
    //logos supports stateful lexing with morph which could be supported
    #[regex("[#A-Za-z/][A-Za-z0-9,._+*#?@-]*")]
    PropNodeName(&'source str),

    #[regex("0x[0-9a-fA-F]+")]
    HexValue(&'source str),

    #[regex("-?[0-9]+")]
    Number(&'source str),

    //TODO fixthis regex
    #[regex(r#""([^"\\\x00-\x1F]|\\(["\\bnfrt/]|u[a-fA-F0-9]{4}))*""#, |lex| {
        let slice = lex.slice();
        &slice[1..slice.len()-1]
    })]
    String(&'source str),
}

#[cfg(test)]
mod tests {
    use super::Token;
    use logos::Logos;

    #[test]
    fn test_lex_dts_v1() {
        let dts = "/dts-v1/";

        let mut lexer = Token::lexer(dts);

        let token = lexer.next().unwrap().unwrap();

        assert_eq!(token, Token::DtsV1);
    }

    #[test]
    fn test_lex_line_comment() {
        let dts = "/dts-v1/;\n// some comment \n/dts-v1/;";
        let tokens = [
            Token::DtsV1,
            Token::SemiColon,
            Token::NewLine,
            Token::Comment(" some comment "),
            Token::DtsV1,
            Token::SemiColon,
        ];

        let mut lexer = Token::lexer(dts);

        for token in tokens {
            let lex_token = lexer.next().unwrap().unwrap();
            assert_eq!(token, lex_token);
        }
    }

    #[test]
    fn test_lex_simple() {
        let src = "soc{}";

        let mut lexer = Token::lexer(src);

        let token = lexer.next().unwrap().unwrap();
        assert_eq!(token, Token::PropNodeName("soc"));

        let token = lexer.next().unwrap().unwrap();
        assert_eq!(token, Token::CurlyOpen);

        let token = lexer.next().unwrap().unwrap();
        assert_eq!(token, Token::CurlyClose);
    }

    #[test]
    fn test_lex_smoke() {
        let dts = include_str!("../dts/test.dts");

        let lexer = Token::lexer(dts);

        for (token, span) in lexer.spanned() {
            assert!(token.is_ok());
            let token = token.unwrap();
            dbg!(token, span);
        }
    }
}
