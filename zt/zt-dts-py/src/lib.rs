use pyo3::prelude::*;

// Owned versions of the parse types (without lifetimes) for Python
#[derive(Debug, Clone)]
#[pyclass]
pub enum PyCell {
    Number(u32),
    PHandle(String),
}

#[pymethods]
impl PyCell {
    fn __repr__(&self) -> String {
        match self {
            PyCell::Number(n) => format!("{}", n),
            PyCell::PHandle(s) => format!("&{}", s),
        }
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }
}

#[derive(Debug, Clone)]
#[pyclass]
pub enum PyPropertyValue {
    ByteArray(Vec<u8>),
    Cells(Vec<PyCell>),
    String(String),
    PHandle(String),
}

#[pymethods]
impl PyPropertyValue {
    fn __repr__(&self) -> String {
        match self {
            PyPropertyValue::ByteArray(b) => {
                format!("[{}]", b.iter().map(|x| format!("{:02x}", x)).collect::<Vec<_>>().join(" "))
            }
            PyPropertyValue::Cells(cells) => {
                format!("<{}>", cells.iter().map(|c| c.__repr__()).collect::<Vec<_>>().join(" "))
            }
            PyPropertyValue::String(s) => format!("\"{}\"", s),
            PyPropertyValue::PHandle(s) => format!("&{}", s),
        }
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }
}

#[derive(Debug, Clone)]
#[pyclass]
pub struct PyDeviceNode {
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub labels: Vec<String>,
    #[pyo3(get)]
    pub properties: Vec<(String, Vec<PyPropertyValue>)>,
    #[pyo3(get)]
    pub child_nodes: Vec<PyDeviceNode>,
}

#[derive(Debug, Clone)]
#[pyclass]
pub enum PyDts {
    DeviceNode(PyDeviceNode),
    DeleteNode(String),
    Include(String),
}

#[pymethods]
impl PyDts {
    #[getter]
    pub fn device_node(&self) -> Option<PyDeviceNode> {
        match self {
            PyDts::DeviceNode(node) => Some(node.clone()),
            _ => None,
        }
    }

    #[getter]
    pub fn delete_node(&self) -> Option<String> {
        match self {
            PyDts::DeleteNode(s) => Some(s.clone()),
            _ => None,
        }
    }

    #[getter]
    pub fn include(&self) -> Option<String> {
        match self {
            PyDts::Include(s) => Some(s.clone()),
            _ => None,
        }
    }

    #[getter]
    pub fn variant_name(&self) -> &'static str {
        match self {
            PyDts::DeviceNode(_) => "DeviceNode",
            PyDts::DeleteNode(_) => "DeleteNode",
            PyDts::Include(_) => "Include",
        }
    }
}

fn convert_cell(cell: &zt_dts::parse::Cell) -> PyCell {
    match cell {
        zt_dts::parse::Cell::Number(n) => PyCell::Number(*n),
        zt_dts::parse::Cell::PHandle(s) => PyCell::PHandle(s.to_string()),
    }
}

fn convert_property_value(val: &zt_dts::parse::PropertyValue) -> PyPropertyValue {
    match val {
        zt_dts::parse::PropertyValue::ByteArray(b) => PyPropertyValue::ByteArray(b.clone()),
        zt_dts::parse::PropertyValue::Cells(cells) => {
            PyPropertyValue::Cells(cells.iter().map(convert_cell).collect())
        }
        zt_dts::parse::PropertyValue::String(s) => PyPropertyValue::String(s.to_string()),
        zt_dts::parse::PropertyValue::PHandle(s) => PyPropertyValue::PHandle(s.to_string()),
    }
}

fn convert_device_node(node: &zt_dts::parse::DeviceNode) -> PyDeviceNode {
    PyDeviceNode {
        name: node.name.to_string(),
        labels: node.labels.iter().map(|s| s.to_string()).collect(),
        properties: node
            .properties
            .iter()
            .map(|(k, v)| {
                (
                    k.to_string(),
                    v.iter().map(convert_property_value).collect(),
                )
            })
            .collect(),
        child_nodes: node
            .child_nodes
            .values()
            .map(convert_device_node)
            .collect(),
    }
}

fn convert_dts(dts: &zt_dts::parse::Dts) -> PyDts {
    match dts {
        zt_dts::parse::Dts::DeviceNode(node) => PyDts::DeviceNode(convert_device_node(node)),
        zt_dts::parse::Dts::DeleteNode(s) => PyDts::DeleteNode(s.to_string()),
        zt_dts::parse::Dts::Include(s) => PyDts::Include(s.to_string()),
    }
}

/// A Python module implemented in Rust.
#[pymodule]
mod zt_dts_py {
    use super::*;

    /// Parse a device tree source string and return parsed items.
    #[pyfunction]
    fn parse(src: &str) -> PyResult<Vec<PyDts>> {
        match zt_dts::parse_str(src) {
            Ok(items) => Ok(items.iter().map(convert_dts).collect()),
            Err((msg, span)) => Err(PyErr::new::<pyo3::exceptions::PySyntaxError, _>(
                format!("Parse error at {:?}: {}", span, msg),
            )),
        }
    }

    #[pymodule_export]
    use super::{PyDts, PyDeviceNode, PyPropertyValue, PyCell};
}
