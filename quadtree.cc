/*
 * Author: Dan Lin, danielin@uw.edu
 */

#include "quadtree.h"

#include <cairo.h>
#include <cmath>
#include <iostream>
#include <queue>
#include <vector>

using namespace std;

const double kScaleFactor = 100;

//-----------------------------------------------------------------------------

template<typename T>
struct NodeDistanceCompare {
  bool operator()(const pair<long double, T>& val1,
                  const pair<long double, T>& val2) const {
    return val1.first > val2.first;
  }
};

//-----------------------------------------------------------------------------

long double EuclideanDistance(const QuadTree::Point& p1,
                          const QuadTree::Point& p2) {
  long double x_d = p1.x - p2.x;
  long double y_d = p1.y - p2.y;
  return sqrt(x_d * x_d + y_d * y_d);
}

//-----------------------------------------------------------------------------

struct QuadTree::Node {
  typedef shared_ptr<Node> Ptr;

  Node(long double side_length_p) :
    side_length(side_length_p),
    contained_point(nullptr) {
  }

  // Top left corner of this bounding box.
  Point bound_box_corner;

  // Side length of this bounding box.
  long double side_length;

  // Singular point contained by this node. Should only be set if this is a
  // non-empty leaf node. Will be NULL otherwise.
  unique_ptr<Point> contained_point;

  // Children of this node. If this node is a inner node this will contain a
  // pointer to the Node for each quadrant.
  vector<shared_ptr<Node>> children_vec;

  // Distance from this bounding box to point 'p'.
  long double DistanceFrom(const Point& p);

  // Whether the bounding box for this node contains the point 'p'.
  bool BoxContains(const Point& p);

  // Whether x or y coordinate of 'p' falls within the our bounding box range.
  bool WithinXBounds(const Point& p);
  bool WithinYBounds(const Point& p);

  // Util method to flip 'y' coordinate across line at y coordinate
  // 'flip_axis_y'. We need to do this as low y coordinates are drawn at the
  // top in cairo.
  long double FlipY(long double y, long double flip_axis_y);

  // Draws the bounding box and any bounded point to 'cairo_ctx'. 'offset'
  // passed in so negative points are correctly drawn.
  void Draw(cairo_t *cairo_ctx, long double offset);

  // Split this node into quads. Place the contained point in one of the split
  // quads if needed.
  void Split();
};

//-----------------------------------------------------------------------------
//                          QuadTree::Node
//-----------------------------------------------------------------------------

long double QuadTree::Node::DistanceFrom(const Point& p) {
  // First rule out that p is within this box.
  const bool within_x = WithinXBounds(p);
  const bool within_y = WithinYBounds(p);

  // If this point lies inside our box then return 0.
  if (within_x && within_y) {
    return 0;
  }

  // X coordinate is aligned we can just return the y difference.
  if (within_x) {
    if (p.y <  bound_box_corner.y) {
      // This point lies below so you get distance from lowest side.
      return (bound_box_corner.y - side_length) - p.y;
    } else {
      return p.y - bound_box_corner.y;
    }
  }

  if (within_y) {
    if (p.x >  bound_box_corner.x) {
      // This point lies to the right of our box use the righ side.
      return p.x - (bound_box_corner.x + side_length);
    } else {
      return bound_box_corner.x - p.x;
    }
  }

  // Now we have to compute the distance in absolute terms from one of our
  // box corner points, which ever corner is closest.
  Point box_point = bound_box_corner;
  if (p.x > bound_box_corner.x) {
    box_point.x += side_length;
  }

  if (p.y < bound_box_corner.y) {
    box_point.y -= side_length;
  }
  return EuclideanDistance(box_point, p);
}

//-----------------------------------------------------------------------------


bool QuadTree::Node::BoxContains(const Point& p) {
  return WithinXBounds(p) && WithinYBounds(p);
}

//-----------------------------------------------------------------------------

bool QuadTree::Node::WithinXBounds(const Point& p) {
  return bound_box_corner.x <= p.x &&
         (bound_box_corner.x + side_length > p.x);
}

//-----------------------------------------------------------------------------

bool QuadTree::Node::WithinYBounds(const Point& p) {
  return bound_box_corner.y > p.y &&
         (bound_box_corner.y - side_length <= p.y);
}

//-----------------------------------------------------------------------------

void QuadTree::Node::Split() {
  assert(children_vec.empty());
  long double new_side_length = side_length / 2.0;
  for (int ii = 0; ii < 4; ++ii) {
    children_vec.push_back(make_shared<Node>(new_side_length));
  }

  // Update the 4 quadrant corners.
  children_vec[0]->bound_box_corner = bound_box_corner;
  children_vec[1]->bound_box_corner =
    Point(bound_box_corner.x + new_side_length, bound_box_corner.y);
  children_vec[2]->bound_box_corner =
    Point(bound_box_corner.x + new_side_length,
          bound_box_corner.y - new_side_length);
  children_vec[3]->bound_box_corner =
    Point(bound_box_corner.x,
          bound_box_corner.y - new_side_length);

  if (contained_point) {
    //cout << "Contained point: " << *contained_point << endl;
    for (int ii = 0; ii < children_vec.size(); ++ii) {
      //cout << "Bound box corner: " << children_vec[ii]->bound_box_corner << endl;
      if (children_vec[ii]->BoxContains(*contained_point)) {
        children_vec[ii]->contained_point = move(contained_point);
        contained_point = nullptr;
        break;
      }
    }
    // A child should now hold our previously contained point.
    assert(!contained_point);
  }
}

//-----------------------------------------------------------------------------

void QuadTree::Node::Draw(cairo_t *const cairo_ctx, const long double offset) {
  long double x = bound_box_corner.x + offset;
  long double y = bound_box_corner.y + offset;

  cout << "Drawing box with corner " << x << " " << y << " "
       << "length: " << side_length << endl;
  assert(x >= 0);
  assert(y >= 0);
  // Set line color to black.
  cairo_set_source_rgb(cairo_ctx, 0, 0, 0);
  cairo_set_line_width(cairo_ctx, 0.1);
  cairo_move_to(cairo_ctx, x, FlipY(y, offset));
  cairo_line_to(cairo_ctx, x + side_length, FlipY(y, offset));
  cairo_line_to(cairo_ctx, x + side_length, FlipY(y - side_length, offset));
  cairo_line_to(cairo_ctx, x, FlipY(y - side_length, offset));
  cairo_line_to(cairo_ctx, x, FlipY(y, offset));
  cairo_stroke(cairo_ctx);
  if (contained_point) {
    cout << "Drawing " << contained_point->x + offset
         << ", " << contained_point->y + offset
         << endl;
    cairo_move_to(cairo_ctx,
                  contained_point->x + offset,
                  FlipY(contained_point->y + offset, offset));
    cairo_set_source_rgb(cairo_ctx, 0, 0, 200);
    cairo_arc (cairo_ctx,
               contained_point->x + offset,
               FlipY(contained_point->y + offset, offset),
               0.1 /* r */,
               0, 2*M_PI);
    cairo_fill (cairo_ctx);

    //cairo_set_line_width(cairo_ctx, 1);
//cairo_close_path (cairo_ctx);
    //cairo_line_to(cairo_ctx, contained_point->x, contained_point->y);
 //   cairo_stroke(cairo_ctx);
  }
  for (int ii = 0; ii < children_vec.size(); ++ii) {
    children_vec[ii]->Draw(cairo_ctx, offset);
  }
}

//-----------------------------------------------------------------------------

long double QuadTree::Node::FlipY(const long double y,
                                  const long double flip_axis_y) {

  long double diff = flip_axis_y - y;
  return flip_axis_y + diff;
}

//-----------------------------------------------------------------------------
//                            QuadTree
//-----------------------------------------------------------------------------

QuadTree::QuadTree(const long double max_size) :
  root_node_(make_shared<Node>(max_size)) {
  assert(max_size > 0);
  root_node_->bound_box_corner.x = (max_size / 2) * -1;
  root_node_->bound_box_corner.y = max_size / 2;
}

//-----------------------------------------------------------------------------

void QuadTree::InsertPoint(const Point& p) {
  //cout << p << endl;
  assert(abs(p.x) <= root_node_->side_length / 2);
  assert(abs(p.y) <= root_node_->side_length / 2);

  Node::Ptr leaf_node = Seek(p, root_node_);
  assert(leaf_node);

  // We now have the current leaf node. If a node is not already contained we
  // can simply add our point and return. Otherwise we need to continue
  // splitting until we have an empty quad for our new point.
  while (leaf_node->contained_point) {
    if (*leaf_node->contained_point == p) {
      // Point already inserted.
      return;
    }
    leaf_node->Split();
    leaf_node = Seek(p, leaf_node);
  }
  leaf_node->contained_point = make_unique<Point>(p.x, p.y);
}

//-----------------------------------------------------------------------------

shared_ptr<QuadTree::Node> QuadTree::Seek(const Point& p,
                                          const Node::Ptr& start_node) {

  shared_ptr<Node> next_node = start_node;
  while (next_node && !IsLeaf(next_node)) {
    shared_ptr<Node> new_next = nullptr;
    const vector<Node::Ptr> children_vec = next_node->children_vec;
    for (int ii = 0; ii < children_vec.size(); ++ii) {
      if (children_vec[ii] && children_vec[ii]->BoxContains(p)) {
        new_next = children_vec[ii];
        break;
      }
    }
    next_node = new_next;
  }
  return next_node;
}

//-----------------------------------------------------------------------------

QuadTree::Point QuadTree::QueryClosest(const Point& p) {
  Point closest_point;
  long double closest_distance = numeric_limits<long double>::max();

  // Maintain a min heap for each node we are currently searching. We will
  // always search the closest nodes first. Once the next node in the heap is
  // further from p than our current closest we can return.
  priority_queue<pair<long double, Node::Ptr>,
                 vector<pair<long double, Node::Ptr>>,
                 NodeDistanceCompare<Node::Ptr>> min_heap;
  min_heap.push(
    pair<long double, Node::Ptr>(root_node_->DistanceFrom(p), root_node_));

  cout << "Looking for point closest to: " << p << endl;
  while (min_heap.size()) {
    pair<long double, Node::Ptr> next_node = min_heap.top();
    min_heap.pop();
    if (next_node.first > closest_distance) {
      // No more nodes that are closer than our best.
      break;
    }

    cout << "Checking node with box corner: "
         << next_node.second->bound_box_corner << " and width "
         << next_node.second->side_length << endl;

    if (!IsLeaf(next_node.second)) {
      const vector<Node::Ptr>& children_vec = next_node.second->children_vec;
      for (int ii = 0; ii < next_node.second->children_vec.size(); ++ii) {
        assert(children_vec[ii]);
        min_heap.push(
          pair<long double, Node::Ptr>(children_vec[ii]->DistanceFrom(p),
                                   children_vec[ii]));
      }
      continue;
    }

    if (!next_node.second->contained_point) {
      // No point contained in this box.
      continue;
    }

    // Update our best if this point is closest.
    const long double distance =
      EuclideanDistance(*next_node.second->contained_point, p);
    cout << "Distance " << distance << " to point "
         << *next_node.second->contained_point << endl;
    if (distance < closest_distance) {
      closest_point = *next_node.second->contained_point;
      closest_distance = distance;
      cout << "Closest Distance: " << closest_distance << endl;
      cout << "New best: " << closest_point << endl;
    }
  }
  return closest_point;
}

//-----------------------------------------------------------------------------

void QuadTree::Draw() {
  cairo_surface_t *surface =
    cairo_image_surface_create(
      CAIRO_FORMAT_ARGB32,
      1 + root_node_->side_length * kScaleFactor/* width */,
      1 + root_node_->side_length * kScaleFactor/* height */);
  cairo_t *cairo_ctx = cairo_create(surface);

  // We will scale and invert y axis.
  cairo_scale(cairo_ctx, kScaleFactor, kScaleFactor);
  cairo_set_source_rgb (cairo_ctx, 0, 0, 0);
  root_node_->Draw(cairo_ctx, (root_node_->side_length / 2));
  cairo_status_t status =
    cairo_surface_write_to_png(surface, "qtree.png");
  cout <<"Status: " << cairo_status_to_string(status) << endl;
}

//-----------------------------------------------------------------------------

bool QuadTree::IsLeaf(const Node::Ptr& node) {
  assert(node);

  return node->contained_point != nullptr || node->children_vec.empty();
}

//-----------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const QuadTree::Point& p) {
  os << "(" << p.x << ", " << p.y << ")";
  return os;
}

//-----------------------------------------------------------------------------
