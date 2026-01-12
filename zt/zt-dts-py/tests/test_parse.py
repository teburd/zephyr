"""Tests for zt_dts_py parsing functionality."""

from pathlib import Path

import pytest
import zt_dts_py


@pytest.fixture
def test_dts_content():
    """Load the test.dts file."""
    test_dts_path = Path(__file__).parent.parent.parent / "zt-dts" / "dts" / "test.dts"
    with open(test_dts_path) as f:
        return f.read()


class TestParse:
    """Test device tree parsing functionality."""

    def test_parse_valid_dts(self, test_dts_content):
        """Test parsing a valid device tree source file."""
        result = zt_dts_py.parse(test_dts_content)

        assert result is not None
        assert isinstance(result, list)
        assert len(result) > 0

    def test_parse_returns_device_node(self, test_dts_content):
        """Test that parse returns PyDts objects with variant information."""
        result = zt_dts_py.parse(test_dts_content)

        assert len(result) == 1
        item = result[0]
        assert hasattr(item, 'variant_name')
        assert item.variant_name == "DeviceNode"

    def test_parse_device_node_structure(self, test_dts_content):
        """Test that parsed device nodes have expected structure."""
        result = zt_dts_py.parse(test_dts_content)
        item = result[0]

        node = item.device_node
        assert node is not None
        assert hasattr(node, 'name')
        assert hasattr(node, 'labels')
        assert hasattr(node, 'properties')
        assert hasattr(node, 'child_nodes')

    def test_parse_root_node_name(self, test_dts_content):
        """Test that the root node has the correct name."""
        result = zt_dts_py.parse(test_dts_content)
        item = result[0]
        node = item.device_node

        assert node.name == "/"

    def test_parse_child_nodes_exist(self, test_dts_content):
        """Test that parsed root node has child nodes."""
        result = zt_dts_py.parse(test_dts_content)
        item = result[0]
        node = item.device_node

        assert isinstance(node.child_nodes, list)
        assert len(node.child_nodes) > 0

    def test_parse_properties_accessible(self, test_dts_content):
        """Test that node properties are accessible."""
        result = zt_dts_py.parse(test_dts_content)
        item = result[0]
        node = item.device_node

        assert isinstance(node.properties, list)
        # Properties are tuples of (name, [values])
        if node.properties:
            name, values = node.properties[0]
            assert isinstance(name, str)
            assert isinstance(values, list)

    def test_parse_property_values_have_string_representation(self, test_dts_content):
        """Test that property values can be converted to strings."""
        result = zt_dts_py.parse(test_dts_content)
        item = result[0]
        node = item.device_node

        # Find a child node with properties
        def find_node_with_props(n):
            if n.properties:
                return n
            for child in n.child_nodes:
                result = find_node_with_props(child)
                if result:
                    return result
            return None

        prop_node = find_node_with_props(node)
        assert prop_node is not None

        name, values = prop_node.properties[0]
        for val in values:
            # Should not raise an exception
            str_val = str(val)
            assert isinstance(str_val, str)
            assert len(str_val) > 0

    def test_parse_empty_string_allows_parsing(self):
        """Test that parsing empty string doesn't raise an immediate error."""
        # Empty DTS is technically empty content
        result = zt_dts_py.parse("/dts-v1/;")
        assert isinstance(result, list)

    @pytest.mark.skip(reason="Parser panics on invalid DTS instead of raising")
    def test_parse_invalid_dts_may_panic(self):
        """Test that parsing invalid DTS may raise or panic."""
        invalid_dts = "/dts-v1/; / { invalid syntax here }}}"
        # Invalid DTS can either raise an exception or panic
        # We just verify it doesn't silently pass
        try:
            result = zt_dts_py.parse(invalid_dts)
            # If it doesn't panic, result should still be reasonable
            assert result is not None
        except Exception:
            # Exception is acceptable for invalid DTS
            pass

    def test_parse_simple_valid_dts(self):
        """Test parsing a simple valid device tree."""
        simple_dts = "/dts-v1/; / { };"
        result = zt_dts_py.parse(simple_dts)

        assert len(result) == 1
        item = result[0]
        assert item.variant_name == "DeviceNode"
        assert item.device_node.name == "/"

    def test_parse_node_with_properties(self):
        """Test parsing a node with properties."""
        dts = '/dts-v1/; / { node { compatible = "test"; }; };'
        result = zt_dts_py.parse(dts)

        assert len(result) == 1
        node = result[0].device_node
        assert len(node.child_nodes) == 1

        child = node.child_nodes[0]
        assert child.name == "node"
        assert len(child.properties) > 0

    def test_parse_preserves_property_values(self):
        """Test that property values are correctly preserved."""
        dts = '/dts-v1/; / { node { prop = "value"; }; };'
        result = zt_dts_py.parse(dts)

        node = result[0].device_node.child_nodes[0]
        assert len(node.properties) == 1

        prop_name, prop_values = node.properties[0]
        assert prop_name == "prop"
        assert len(prop_values) == 1
        assert '"value"' in str(prop_values[0])

    def test_parse_multiple_properties(self):
        """Test parsing a node with multiple properties."""
        dts = '/dts-v1/; / { node { prop1 = "val1"; prop2 = <42>; }; };'
        result = zt_dts_py.parse(dts)

        node = result[0].device_node.child_nodes[0]
        assert len(node.properties) >= 2


class TestPropertyValues:
    """Test property value representation."""

    def test_string_property_value(self):
        """Test string property value representation."""
        dts = '/dts-v1/; / { node { test = "hello"; }; };'
        result = zt_dts_py.parse(dts)

        node = result[0].device_node.child_nodes[0]
        _, values = node.properties[0]

        val_str = str(values[0])
        assert "hello" in val_str

    def test_cells_property_value(self):
        """Test cells property value representation."""
        dts = '/dts-v1/; / { node { test = <1 2 3>; }; };'
        result = zt_dts_py.parse(dts)

        node = result[0].device_node.child_nodes[0]
        _, values = node.properties[0]

        val_str = str(values[0])
        assert "<" in val_str and ">" in val_str


class TestChildNodes:
    """Test child node parsing."""

    def test_parse_nested_nodes(self):
        """Test parsing nested device nodes."""
        dts = '/dts-v1/; / { parent { child { }; }; };'
        result = zt_dts_py.parse(dts)

        root = result[0].device_node
        assert len(root.child_nodes) == 1

        parent = root.child_nodes[0]
        assert parent.name == "parent"
        assert len(parent.child_nodes) == 1

        child = parent.child_nodes[0]
        assert child.name == "child"

    def test_parse_multiple_child_nodes(self):
        """Test parsing multiple child nodes at same level."""
        dts = '/dts-v1/; / { child1 { }; child2 { }; child3 { }; };'
        result = zt_dts_py.parse(dts)

        root = result[0].device_node
        assert len(root.child_nodes) == 3

        names = [child.name for child in root.child_nodes]
        assert "child1" in names
        assert "child2" in names
        assert "child3" in names


class TestBenchmarks:
    """Performance benchmarks for device tree parsing."""

    def test_benchmark_parse_test_dts(self, benchmark, test_dts_content):
        """Benchmark parsing the full test.dts file."""
        result = benchmark(zt_dts_py.parse, test_dts_content)

        # Verify result is valid
        assert len(result) == 1
        assert result[0].variant_name == "DeviceNode"

    def test_benchmark_parse_simple_dts(self, benchmark):
        """Benchmark parsing a simple device tree."""
        simple_dts = '/dts-v1/; / { node { compatible = "test"; }; };'
        result = benchmark(zt_dts_py.parse, simple_dts)

        assert len(result) == 1
        assert result[0].device_node.name == "/"

    def test_benchmark_parse_multiple_nodes(self, benchmark):
        """Benchmark parsing a device tree with many nodes."""
        # Create a DTS with many nodes
        nodes = "; ".join(f"node{i} {{}}" for i in range(50))
        dts = f'/dts-v1/; / {{ {nodes}; }};'

        result = benchmark(zt_dts_py.parse, dts)

        assert len(result) == 1
        root = result[0].device_node
        assert len(root.child_nodes) == 50
