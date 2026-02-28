use crate::lex::{Result, Token};
use log::warn;
use logos::Lexer;
use std::ops::Range;
use std::str::FromStr;

use std::collections::HashMap;

#[derive(Debug, PartialEq, Eq)]
pub enum Cell<'source> {
    Number(u32),
    PHandle(&'source str),
}

#[derive(Debug, PartialEq, Eq)]
pub enum PropertyValue<'source> {
    ByteArray(Vec<u8>),
    Cells(Vec<Cell<'source>>),
    String(&'source str),
    PHandle(&'source str),
}

#[derive(Debug, PartialEq, Eq)]
pub struct DeviceNode<'source> {
    pub name: &'source str,
    pub labels: Vec<&'source str>,
    pub properties: HashMap<&'source str, Vec<PropertyValue<'source>>>,
    pub child_nodes: HashMap<&'source str, Self>,
}

#[derive(Debug, PartialEq, Eq)]
pub enum Dts<'source> {
    DeviceNode(DeviceNode<'source>),
    DeleteNode(&'source str),
    Include(&'source str),
}

fn parse_hex_u8<'source>(input: &'source str, _span: Range<usize>) -> Result<u8> {
    let trimmed = input.trim_start_matches("0x");
    u8::from_str_radix(trimmed, 16).or(Ok(0u8))
}

fn parse_hex_u32<'source>(input: &'source str, _span: Range<usize>) -> Result<u32> {
    let trimmed = input.trim_start_matches("0x");
    u32::from_str_radix(trimmed, 16).or(Ok(0u32))
}

fn parse_bytes<'source>(lexer: &mut Lexer<'source, Token<'source>>) -> Result<Vec<u8>> {
    let mut bytes: Vec<u8> = Vec::new();
    while let Some(token) = lexer.next() {
        match token {
            Ok(Token::HexValue(raw_cell)) => {
                let byte: u8 = parse_hex_u8(raw_cell, lexer.span())?;
                bytes.push(byte);
            }
            Ok(Token::Number(raw_cell)) => {
                let byte: u8 = u8::from_str(raw_cell).unwrap();
                bytes.push(byte)
            }
            Ok(Token::NewLine) => (),
            Ok(Token::WhiteSpace) => (),
            Ok(Token::SquareClose) => return Ok(bytes),
            Ok(tok) => {
                return Err((
                    format!("unexpected token {:?} here, context: bytes", tok),
                    lexer.span(),
                ));
            }
            Err(_) => todo!(),
        }
    }
    Ok(Vec::new())
}

fn parse_cells<'source>(lexer: &mut Lexer<'source, Token<'source>>) -> Result<Vec<Cell<'source>>> {
    let mut cells: Vec<Cell> = Vec::new();
    while let Some(token) = lexer.next() {
        match token {
            Ok(Token::Number(raw_cell)) => {
                let cell: u32 = u32::from_str(raw_cell).unwrap();
                cells.push(Cell::Number(cell));
            }
            Ok(Token::HexValue(raw_cell)) => {
                let cell: u32 = parse_hex_u32(raw_cell, lexer.span())?;
                cells.push(Cell::Number(cell));
            }
            Ok(Token::PHandle(phandle)) => {
                cells.push(Cell::PHandle(phandle));
            }
            Ok(Token::Comment(_)) => (),
            Ok(Token::WhiteSpace) => (),
            Ok(Token::NewLine) => (),
            Ok(Token::AngleClose) => return Ok(cells),
            Ok(tok) => {
                return Err((
                    format!("unexpected token {:?} here, context: cells", tok),
                    lexer.span(),
                ));
            }
            Err(_) => todo!(),
        }
    }
    unreachable!();
}

fn parse_value<'source>(
    lexer: &mut Lexer<'source, Token<'source>>,
) -> Result<Vec<PropertyValue<'source>>> {
    // every property is potentially an array of composite values
    let mut values: Vec<PropertyValue> = Vec::new();

    while let Some(token) = lexer.next() {
        match token {
            Ok(Token::SemiColon) => return Ok(values),
            Ok(Token::Comma) => (),
            Ok(Token::NewLine) => (),
            Ok(Token::WhiteSpace) => (),
            Ok(Token::SquareOpen) => {
                let bytes = parse_bytes(lexer)?;
                values.push(PropertyValue::ByteArray(bytes));
            }
            Ok(Token::AngleOpen) => {
                let cells = parse_cells(lexer)?;
                values.push(PropertyValue::Cells(cells));
            }
            Ok(Token::String(str)) => values.push(PropertyValue::String(str)),
            Ok(Token::PHandle(phandle)) => values.push(PropertyValue::PHandle(phandle)),
            Ok(tok) => {
                return Err((
                    format!("unexpected token {:?} here, context: value", tok),
                    lexer.span(),
                ));
            }
            Err(err) => {
                return Err((
                    format!("error parsing {:?}, context: value", err),
                    lexer.span(),
                ));
            }
        }
    }
    unreachable!();
}

enum PropNode<'source> {
    Node(DeviceNode<'source>),
    Property(Vec<PropertyValue<'source>>),
}

fn parse_prop_node<'source>(
    lexer: &mut Lexer<'source, Token<'source>>,
    name: &'source str,
) -> Result<PropNode<'source>> {
    let mut peeker = lexer.peekable();

    // We have to look ahead for assign or curly open to decide
    while let Some(token) = peeker.next() {
        match token {
            Ok(Token::NewLine) => continue,
            Ok(Token::WhiteSpace) => continue,
            Ok(Token::Comment(_)) => continue,
            Ok(Token::CurlyOpen) => {
                let node = parse_node(lexer, Vec::new(), name)?;
                return Ok(PropNode::Node(node));
            }
            Ok(Token::Assign) => {
                let prop_val = parse_value(lexer)?;
                return Ok(PropNode::Property(prop_val));
            }
            Ok(tok) => {
                warn!("unused token {:?}, context: prop node", tok);
                continue;
            }
            Err(_) => todo!(),
        }
    }
    panic!()
}

fn parse_node<'source>(
    lexer: &mut Lexer<'source, Token<'source>>,
    labels: Vec<&'source str>,
    name: &'source str,
) -> Result<DeviceNode<'source>> {
    let mut curly_count = 0;

    //TODO use a pre (or arena?) allocated device node, don't constantly alloc here...
    let mut node = DeviceNode {
        name: name,
        labels: labels.clone(),
        properties: HashMap::new(),
        child_nodes: HashMap::new(),
    };

    while let Some(token) = lexer.next() {
        match token {
            // TODO Parse node body properties and subnodes
            Ok(Token::NewLine) => continue,
            Ok(Token::WhiteSpace) => continue,
            Ok(Token::Comment(_)) => continue,
            Ok(Token::CurlyOpen) => {
                curly_count += 1;
                continue;
            }
            Ok(Token::CurlyClose) => {
                curly_count -= 1;

                if curly_count <= 0 {
                    break;
                }
            }
            Ok(Token::SemiColon) => continue,
            Ok(Token::Label(label)) => {
                let child_node = parse_node_labels(lexer, label)?;
                node.child_nodes.insert(child_node.name, child_node);
                continue;
            }
            Ok(Token::PropNodeName(name)) => {
                let prop_node = parse_prop_node(lexer, name)?;
                match prop_node {
                    PropNode::Property(prop) => {
                        node.properties.insert(name, prop);
                        continue;
                    }
                    PropNode::Node(child_node) => {
                        node.child_nodes.insert(name, child_node);
                        continue;
                    }
                }
            }
            Ok(tok) => {
                println!(
                    "FIXME unhandled token in parse_node {:?}, span: {:?}",
                    tok,
                    lexer.span()
                );
                continue;
            }
            Err(err) => {
                return Err((
                    format!("error parsing {:?}, context: value", err),
                    lexer.span(),
                ));
            }
        }
    }

    return Ok(node);
}

// Parse found a label, lets parse labels and a node name
fn parse_node_labels<'source>(
    lexer: &mut Lexer<'source, Token<'source>>,
    first_label: &'source str,
) -> Result<DeviceNode<'source>> {
    let mut labels = vec![first_label];

    while let Some(token) = lexer.next() {
        match token {
            Ok(Token::WhiteSpace) => continue,
            Ok(Token::Comment(_)) => continue,
            Ok(Token::Label(label)) => {
                labels.push(label);
                continue;
            }
            Ok(Token::PropNodeName(name)) => {
                let node = parse_node(lexer, labels, name)?;
                return Ok(node);
            }
            _ => panic!(),
        }
    }
    panic!()
}

pub fn parse_root<'source>(
    lexer: &mut Lexer<'source, Token<'source>>,
) -> Result<Vec<Dts<'source>>> {
    let mut values: Vec<Dts> = Vec::new();

    // Expect nothing but comments and white space until get to DtsV1
    while let Some(token) = lexer.next() {
        match token {
            Ok(Token::WhiteSpace) => continue,
            Ok(Token::NewLine) => continue,
            Ok(Token::Comment(_)) => continue,
            Ok(Token::DtsV1) => break, // TODO check for semicolon?
            Ok(tok) => {
                return Err((
                    format!("unexpected token {:?}, here, context: root", tok),
                    lexer.span(),
                ));
            }
            Err(err) => {
                return Err((
                    format!("lexer error {:?}, context: root", err),
                    lexer.span(),
                ));
            }
        }
    }

    // Expect nothing but while space and a semi colon after DtsV1
    while let Some(token) = lexer.next() {
        match token {
            Ok(Token::WhiteSpace) => continue,
            Ok(Token::NewLine) => continue,
            Ok(Token::SemiColon) => break,
            Ok(tok) => {
                return Err((
                    format!("unexpected token {:?}, here, context: root", tok),
                    lexer.span(),
                ));
            }
            Err(err) => {
                return Err((
                    format!("lexer error {:?}, context: root", err),
                    lexer.span(),
                ));
            }
        }
    }

    // Parse all root nodes and dts instructions into the Dts tree
    while let Some(token) = lexer.next() {
        match token {
            Ok(Token::NewLine) => continue,
            Ok(Token::WhiteSpace) => continue,
            Ok(Token::Include(_include_dts)) => continue, // TODO
            Ok(Token::DeleteNode(_delete_node)) => continue, // TODO
            Ok(Token::Label(label)) => {
                let node = parse_node_labels(lexer, label)?;
                values.push(Dts::DeviceNode(node));
            }
            Ok(Token::PropNodeName(name)) => {
                let node = parse_node(lexer, Vec::new(), name)?;
                values.push(Dts::DeviceNode(node));
            }
            Ok(Token::SemiColon) => continue,
            Ok(tok) => {
                return Err((
                    format!("unexpected token {:?}, here, context: root", tok),
                    lexer.span(),
                ));
            }
            Err(err) => {
                return Err((
                    format!("lexer error {:?}, context: root", err),
                    lexer.span(),
                ));
            }
        }
    }

    Ok(values)
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::lex::Token;
    use logos::Logos;

    #[test]
    fn test_parse_property() {
        let test_scenarios = [
            ("<32>;", vec![PropertyValue::Cells(vec![Cell::Number(32)])]),
            ("\"Hello\";", vec![PropertyValue::String("Hello")]),
            //TODO fix hex parsing (
            //    "[0xFF 0x12 0xBA];",
            //    vec![PropertyValue::ByteArray(vec![0xff, 0x12, 0xba])],
            //),
            (
                "\"Hello\", \"world\", <42>;",
                vec![
                    PropertyValue::String("Hello"),
                    PropertyValue::String("world"),
                    PropertyValue::Cells(vec![Cell::Number(42)]),
                ],
            ),
        ];

        for (input, expected) in test_scenarios {
            let mut lexer = Token::lexer(input);

            let val = parse_value(&mut lexer);
            dbg!(&val);
            assert!(val.is_ok());
            let val = val.unwrap();
            dbg!(&val);
            assert_eq!(val, expected);
        }
    }

    #[test]
    fn test_parse_node_labels() {
        let test_scenarios = [
            "
            /dts-v1/;

            alias1: alias2: node@feeddeaf {
                    compatible = \"myvnd,myip-block\";
                    reg = <12345>;
                    #size-cells = <1>;
                    #address-cells = <1>;
                    alias3: alias4: node2@12 {
                        reg = <12>;
                    };
                    node3 {
                        reg = <13>;
                    };
                };
            ",
            include_str!("../dts/test.dts"),
        ];

        for input in test_scenarios {
            let mut lexer = Token::lexer(input);

            let val = parse_root(&mut lexer);
            dbg!(&val);
            dbg!(val.is_ok());
            let val = val.unwrap();
            dbg!("devicetree {:?}", &val);
        }
    }
}
