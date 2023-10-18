import unittest
import numpy as np
from main import A, v, n, adjacency_matrix, start_state, end_state, dijkstra

class TestMainCode(unittest.TestCase):
    # Test Case 1: Verify that v_new is calculated correctly
    def test_calculate_v_new(self):
        v_new = np.dot(np.linalg.matrix_power(A, n), v)
        expected_v_new = np.array([[15.70251994 ,16.48371447 ,16.48501367, 15.66192859, 16.4460001 ,16.48486203,
 16.40897221, 16.44949818, 17.19474538, 17.26212923, 17.23174603 ,16.52247503 , 16.48528922 ,16.48202533 ,15.6993175 ]])
        self.assertTrue(np.allclose(v_new, expected_v_new, atol=1e-8))

    # Test Case 2: Verify that the shortest path is found correctly
    def test_find_shortest_path(self):
        shortest_path = dijkstra(adjacency_matrix, start_state, end_state)
        expected_path = [9,14]
        self.assertEqual(shortest_path, expected_path)

    # Test Case 3: Verify the behavior when n=0
    def test_zero_n(self):
        n = 0
        v_new = np.dot(np.linalg.matrix_power(A, n), v)
        expected_v_new = v
        assert np.allclose(v_new, expected_v_new)

    # Test Case 4: Verify the behavior when A is the identity matrix
    def test_identity_matrix(self):
        A_identity = np.identity(len(A))
        v_new = np.dot(np.linalg.matrix_power(A_identity, n), v)
        expected_v_new = v
        assert np.allclose(v_new, expected_v_new)

    # Test Case 5: Verify the behavior when A has all zero elements
    def test_zero_matrix(self):
        A_zero = np.zeros_like(A)
        v_new = np.dot(np.linalg.matrix_power(A_zero, n), v)
        expected_v_new = v
        assert np.allclose(v_new, expected_v_new)

    # Test Case 6: Verify the behavior when start and end states are the same
    def test_same_start_and_end(self):
        end_state = 1
        shortest_path = dijkstra(adjacency_matrix, start_state, end_state)
        expected_path = [1]
        assert shortest_path == expected_path

    # Test Case 7: Verify the behavior when there is no path from start to end
    def test_no_path(self):
        end_state = 10
        shortest_path = dijkstra(adjacency_matrix, start_state, end_state)
        expected_path = []  # No path exists
        assert shortest_path == expected_path

    # Test Case 8: Verify the behavior when n is negative
    def test_negative_n(self):
        n = -1
        v_new = np.dot(np.linalg.matrix_power(A, n), v)
        expected_v_new = np.linalg.matrix_power(np.linalg.inv(A), abs(n)).dot(v)
        assert np.allclose(v_new, expected_v_new, atol=1e-8)

    # Test Case 9: Verify the behavior when A has elements greater than 1
    def test_large_values_in_A(self):
        A_large = np.array([[2, 0.1], [0.05, 3]])
        n = 3
        v_large = np.array([1, 2])
        v_new = np.dot(np.linalg.matrix_power(A_large, n), v_large)
        expected_v_new = v_large 
        self.assertTrue(np.allclose(v_new, expected_v_new, atol=1e-8))


    # Test Case 10: Verify the behavior with large n
    def test_large_n(self):
        n = 100
        v_new = np.dot(np.linalg.matrix_power(A, n), v)
        assert np.all(v_new >= 0) and np.isclose(np.sum(v_new), 1.0, atol=1e-8)

if __name__ == "__main__":
    unittest.main()
