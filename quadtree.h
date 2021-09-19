/*
 * This file conains the interface for a quadtree a data structure built to
 * peform nearest neighbor searches.
 *
 * Author: Dan Lin, danielin@uw.edu
 */

#ifndef _QUAD_TREE_H_
#define _QUAD_TREE_H_

#include <limits>
#include <memory>
#include <iostream>

class QuadTree {
 public:
  // Constructor.
  QuadTree(long double max_size);

  // Point POD struct.
  struct Point {
    // Default constructor creates points in top left corner of planar space.
    // We will consider a coordinate of -1 to represent infinity as all
    // coordinates are assumed to be non-negative.
    Point() :
      x(0),
      y(0) { }

    Point(const long double x_p, const long double y_p) :
      x(x_p),
      y(y_p) { }

    bool operator==(const Point& other) {
      long double x_diff = abs(other.x - x);
      long double y_diff = abs(other.y - y);
      if (x_diff < 0.000001 && y_diff < 0.000001) {
        return true;
      }
      return false;
    }

    // X coordinate.
    long double x;

    // Y coordinate.
    long double y;
  };

  // Add point 'p' to the quad tree.
  void InsertPoint(const Point& p);

  // Draw all node bounding boxes and current points.
  void Draw();

  // Query the point currently stored in the tree that occurs closest to
  // point ('x', 'y').
  Point QueryClosest(const Point& p );

 private:
  // Forward declaration of our tree node object.
  struct Node;

  // Seek for the leaf node that contains 'p' starting at 'start_node'. Returns
  // the found leaf.
  std::shared_ptr<Node> Seek(
    const Point& p,
    const std::shared_ptr<Node>& start_node);

  // Returns whether a given 'node' is a leaf node.
  bool IsLeaf(const std::shared_ptr<Node>& node);

 private:

  // Root node of our quadtree.
  std::shared_ptr<Node> root_node_;
};

std::ostream& operator<<(std::ostream& os, const QuadTree::Point& p);

#endif // _QUAD_TREE_H_
