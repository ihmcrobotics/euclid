package us.ihmc.euclid.geometry.interfaces;

import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.exceptions.EmptyPolygonException;
import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * Read-only interface for a convex polygon defined in the XY-plane.
 * <p>
 * This implementation of convex polygon is designed for garbage free operations.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface ConvexPolygon2DReadOnly extends Vertex2DSupplier
{
   /**
    * Tests whether the vertices of this polygon are clockwise or counter-clockwise ordered.
    *
    * @return {@code true} if the vertices are clockwise ordered, {@code false} otherwise.
    */
   boolean isClockwiseOrdered();

   /**
    * Tests whether the vertices of this polygon have been updated to ensure {@code this} is a
    * convex polygon.
    * <p>
    * When this polygon is up-to-date, all operations are available. However, when this polygon is
    * out-of-date, most of the operations on {@code this} will throw an
    * {@code OutdatedPolygonException}. To update a polygon, please see
    * {@link ConvexPolygon2DBasics#update()}.
    * </p>
    *
    * @return whether {@code this} is up-to-date or not.
    */
   boolean isUpToDate();

   /**
    * Checks if this polygon has been updated via {@link ConvexPolygon2DBasics#update()} since last
    * time its vertices have been modified.
    *
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default void checkIfUpToDate()
   {
      if (!isUpToDate())
         throw new OutdatedPolygonException("Call the update method before doing any other calculation!");
   }

   /**
    * Gets the number of vertices composing this convex polygon.
    *
    * @return this polygon's size.
    */
   @Override
   int getNumberOfVertices();

   /**
    * Gets a read-only view of this polygon internal vertex buffer.
    * <p>
    * WARNING: The size of the returned list is usually greater than the size of this polygon. The
    * vertices currently composing this polygon are located at the indices &in; [0;
    * {@code this.getNumberOfVertices()}[.
    * </p>
    * <p>
    * The returned list is an unmodifiable list created with
    * {@link Collections#unmodifiableList(List)}.
    * </p>
    *
    * @return a read-only view of this polygon internal vertex buffer.
    */
   List<? extends Point2DReadOnly> getVertexBufferView();

   /**
    * Gets a read-only view of this polygon vertices.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * The returned list is an unmodifiable list created with
    * {@link Collections#unmodifiableList(List)}.
    * </p>
    *
    * @return a read-only view of this polygon vertices.
    */
   default List<? extends Point2DReadOnly> getPolygonVerticesView()
   {
      return getVertexBufferView().subList(0, getNumberOfVertices());
   }

   /**
    * Tests whether this polygon is empty, i.e. it has no vertices.
    *
    * @return {@code true} if this polygon is empty, {@code false} otherwise.
    */
   default boolean isEmpty()
   {
      return getNumberOfVertices() == 0;
   }

   /**
    * Checks if this polygon is empty, i.e. it has no vertices.
    *
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default void checkNonEmpty()
   {
      if (isEmpty())
         throw new EmptyPolygonException("This polygon has no vertex.");
   }

   /**
    * Checks if the given index is contained in the range [0, {@link #getNumberOfVertices()}[.
    *
    * @param index the index to check.
    * @throws IndexOutOfBoundsException if the given index is either negative or greater or equal
    *            than the polygon's number of vertices.
    */
   default void checkIndexInBoundaries(int index)
   {
      if (index < 0)
         throw new IndexOutOfBoundsException("vertexIndex < 0");
      if (index >= getNumberOfVertices())
         throw new IndexOutOfBoundsException("vertexIndex >= numberOfVertices. numberOfVertices = " + getNumberOfVertices());
   }

   /**
    * Tests if any of this polygon's vertices contains a {@link Double#NaN}.
    *
    * @return {@code true} if at least one vertex contains {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   default boolean containsNaN()
   {
      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         if (getVertexBufferView().get(i).containsNaN())
            return true;
      }

      return false;
   }

   /**
    * Gets the read-only reference to the {@code index}<sup>th</sup> vertex of this polygon.
    * <p>
    * Note that the first vertex has the lowest x-coordinate.
    * </p>
    *
    * @param index the index of the vertex in the clockwise ordered list.
    * @return the read-only reference to the vertex.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   @Override
   default Point2DReadOnly getVertex(int index)
   {
      checkIfUpToDate();
      checkNonEmpty();
      checkIndexInBoundaries(index);
      return getVertexBufferView().get(index);
   }

   /**
    * Gets the read-only reference to the vertex located after the {@code index}<sup>th</sup> vertex
    * of this polygon.
    * <p>
    * Note that the first vertex has the lowest x-coordinate.
    * </p>
    *
    * @param index the index of the vertex.
    * @return the read-only reference to the next vertex.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default Point2DReadOnly getNextVertex(int index)
   {
      return getVertex(getNextVertexIndex(index));
   }

   /**
    * Gets the read-only reference to the vertex located before the {@code index}<sup>th</sup>
    * vertex of this polygon.
    * <p>
    * Note that this polygon's vertices are clockwise ordered and that the first vertex has the
    * lowest x-coordinate.
    * </p>
    *
    * @param index the index of the vertex in the clockwise ordered list.
    * @return the read-only reference to the previous vertex.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default Point2DReadOnly getPreviousVertex(int index)
   {
      return getVertex(getPreviousVertexIndex(index));
   }

   /**
    * Gets the read-only reference to the {@code index}<sup>th</sup> vertex of this polygon.
    * <p>
    * This method calculates a new index to emulate a counter-clockwise ordering of this polygon's
    * vertices. The first vertex has the lowest x-coordinate.
    * </p>
    *
    * @param index the index of the vertex in the counter-clockwise ordered list.
    * @return the read-only reference to the vertex.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default Point2DReadOnly getVertexCCW(int index)
   {
      return getVertex(getNumberOfVertices() - 1 - index);
   }

   /**
    * Gets the read-only reference to the vertex located after the {@code index}<sup>th</sup> vertex
    * of this polygon.
    * <p>
    * This method calculates a new index to emulate a counter-clockwise ordering of this polygon's
    * vertices. The first vertex has the lowest x-coordinate.
    * </p>
    *
    * @param index the index of the vertex in the counter-clockwise ordered list.
    * @return the read-only reference to the next vertex.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default Point2DReadOnly getNextVertexCCW(int index)
   {
      return getVertexCCW(getNextVertexIndex(index));
   }

   /**
    * Gets the read-only reference to the vertex located before the {@code index}<sup>th</sup>
    * vertex of this polygon.
    * <p>
    * This method calculates a new index to emulate a counter-clockwise ordering of this polygon's
    * vertices. The first vertex has the lowest x-coordinate.
    * </p>
    *
    * @param index the index of the vertex in the counter-clockwise ordered list.
    * @return the read-only reference to the previous vertex.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default Point2DReadOnly getPreviousVertexCCW(int index)
   {
      return getVertexCCW(getPreviousVertexIndex(index));
   }

   /**
    * Gets the index of the vertex located after the vertex at the index {@code currentVertexIndex}
    * in the list of vertices.
    * <p>
    * Note that this polygon's vertices are clockwise ordered and that the first vertex has the
    * lowest x-coordinate.
    * </p>
    *
    * @param currentVertexIndex the current vertex index.
    * @return the next vertex index.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default int getNextVertexIndex(int currentVertexIndex)
   {
      checkIfUpToDate();
      checkIndexInBoundaries(currentVertexIndex);
      checkNonEmpty();

      if (currentVertexIndex < getNumberOfVertices() - 1)
         return currentVertexIndex + 1;
      else
         return 0;
   }

   /**
    * Gets the index of the vertex located before the vertex at the index {@code currentVertexIndex}
    * in the list of vertices.
    * <p>
    * Note that this polygon's vertices are clockwise ordered and that the first vertex has the
    * lowest x-coordinate.
    * </p>
    *
    * @param currentVertexIndex the current vertex index.
    * @return the before vertex index.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default int getPreviousVertexIndex(int currentVertexIndex)
   {
      checkIfUpToDate();
      checkIndexInBoundaries(currentVertexIndex);
      checkNonEmpty();

      if (currentVertexIndex < 1)
         return getNumberOfVertices() - 1;
      else
         return currentVertexIndex - 1;
   }

   /**
    * Gets the value of this polygon area.
    *
    * @return the are of this polygon.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   double getArea();

   /**
    * Gets the read-only reference to the polygon's centroid.
    * <p>
    * The centroid is not necessarily equal to the average of this polygon's vertices.
    * </p>
    * <p>
    * When viewing a polygon as a physical object with constant density and thickness, the centroid
    * is equivalent to the polygon's center of mass.
    * </p>
    *
    * @return the read-only reference to this polygon's centroid.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   Point2DReadOnly getCentroid();

   /**
    * Gets the read-only reference to this polygon's axis-aligned bounding box.
    *
    * @return this polygon's bounding box.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   BoundingBox2DReadOnly getBoundingBox();

   /**
    * Gets the size along the x-axis of this polygon's bounding box.
    *
    * @return the range on the x-axis of the bounding box.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default double getBoundingBoxRangeX()
   {
      BoundingBox2DReadOnly boundingBox = getBoundingBox();
      return boundingBox.getMaxX() - boundingBox.getMinX();
   }

   /**
    * Gets the size along the y-axis of this polygon's bounding box.
    *
    * @return the range on the y-axis of the bounding box.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default double getBoundingBoxRangeY()
   {
      BoundingBox2DReadOnly boundingBox = getBoundingBox();
      return boundingBox.getMaxPoint().getY() - boundingBox.getMinPoint().getY();
   }

   /**
    * Gets the highest x-coordinate value of the vertices composing this polygon.
    *
    * @return the maximum x-coordinate.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default double getMaxX()
   {
      return getBoundingBox().getMaxX();
   }

   /**
    * Gets the lowest x-coordinate value of the vertices composing this polygon.
    *
    * @return the minimum x-coordinate.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default double getMinX()
   {
      return getBoundingBox().getMinX();
   }

   /**
    * Gets the highest y-coordinate value of the vertices composing this polygon.
    *
    * @return the maximum y-coordinate.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default double getMaxY()
   {
      return getBoundingBox().getMaxY();
   }

   /**
    * Gets the lowest y-coordinate value of the vertices composing this polygon.
    *
    * @return the minimum y-coordinate.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default double getMinY()
   {
      return getBoundingBox().getMinY();
   }

   /**
    * Adds a subset of this polygon's vertices into the given list.
    * <p>
    * The subset consists of the vertices from the vertex at {@code startIndexInclusive} to the
    * vertex {@code endIndexInclusive} while going from start to end in a clockwise order.
    * </p>
    *
    * @param startIndexInclusive the index of the first vertex to add.
    * @param endIndexInclusive the index of the last vertex to add.
    * @param pointListToPack the list into which the vertices are to be added.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default void getPointsInClockwiseOrder(int startIndexInclusive, int endIndexInclusive, List<Point2DReadOnly> pointListToPack)
   {
      checkIfUpToDate();
      checkIndexInBoundaries(startIndexInclusive);
      checkIndexInBoundaries(endIndexInclusive);
      int index = startIndexInclusive;

      while (true)
      {
         pointListToPack.add(getVertex(index));

         if (index == endIndexInclusive)
            break;
         index = getNextVertexIndex(index);
      }
   }

   /**
    * Adds a subset of this polygon's vertices into the given polygon.
    * <p>
    * The subset consists of the vertices from the vertex at {@code startIndexInclusive} to the
    * vertex {@code endIndexInclusive} while going from start to end in a clockwise order.
    * </p>
    *
    * @param startIndexInclusive the index of the first vertex to add.
    * @param endIndexInclusive the index of the last vertex to add.
    * @param polygonToPack the polygon into which the vertices are to be added.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default void getVerticesInClockwiseOrder(int startIndexInclusive, int endIndexInclusive, ConvexPolygon2DBasics polygonToPack)
   {
      checkIfUpToDate();
      checkIndexInBoundaries(startIndexInclusive);
      checkIndexInBoundaries(endIndexInclusive);
      int index = startIndexInclusive;

      while (true)
      {
         polygonToPack.addVertex(getVertex(index));

         if (index == endIndexInclusive)
            break;
         index = getNextVertexIndex(index);
      }
   }

   /**
    * Tests if the given point is inside this polygon or exactly on and edge/vertex of this polygon.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code numberOfVertices == 0}, this method returns {@code false}.
    * <li>if {@code numberOfVertices == 1}, this method returns whether the query and the single
    * vertex are exactly equal.
    * <li>if {@code numberOfVertices == 2}, this method returns whether the query is exactly on the
    * polygons single edge.
    * </ul>
    *
    * @param x the x-coordinate of the query.
    * @param y the y-coordinate of the query.
    * @return {@code true} if the query is inside this polygon, {@code false} otherwise.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default boolean isPointInside(double x, double y)
   {
      return isPointInside(x, y, 0.0);
   }

   /**
    * Determines if the point is inside this convex polygon given the tolerance {@code epsilon}.
    * <p>
    * The sign of {@code epsilon} is equivalent to performing the test against the polygon shrunk by
    * {@code Math.abs(epsilon)} if {@code epsilon < 0.0}, or against the polygon enlarged by
    * {@code epsilon} if {@code epsilon > 0.0}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code numberOfVertices == 0}, this method returns {@code false}.
    * <li>if {@code numberOfVertices == 1}, this method returns {@code false} if {@code epsilon < 0}
    * or if the query is at a distance from the polygon's only vertex that is greater than
    * {@code epsilon}, returns {@code true} otherwise.
    * <li>if {@code numberOfVertices == 2}, this method returns {@code false} if {@code epsilon < 0}
    * or if the query is at a distance from the polygon's only edge that is greater than
    * {@code epsilon}, returns {@code true} otherwise.
    * </ul>
    *
    * @param x the x-coordinate of the query.
    * @param y the y-coordinate of the query.
    * @param epsilon the tolerance to use during the test.
    * @return {@code true} if the query is considered to be inside the polygon, {@code false}
    *         otherwise.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default boolean isPointInside(double x, double y, double epsilon)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.isPoint2DInsideConvexPolygon2D(x, y, getVertexBufferView(), getNumberOfVertices(), isClockwiseOrdered(),
                                                                       epsilon);
   }

   /**
    * Tests if the given point is inside this polygon or exactly on and edge/vertex of this polygon.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code numberOfVertices == 0}, this method returns {@code false}.
    * <li>if {@code numberOfVertices == 1}, this method returns whether the query and the single
    * vertex are exactly equal.
    * <li>if {@code numberOfVertices == 2}, this method returns whether the query is exactly on the
    * polygons single edge.
    * </ul>
    *
    * @param point the query. Not modified.
    * @return {@code true} if the query is inside this polygon, {@code false} otherwise.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default boolean isPointInside(Point2DReadOnly point)
   {
      return isPointInside(point, 0.0);
   }

   /**
    * Determines if the point is inside this convex polygon given the tolerance {@code epsilon}.
    * <p>
    * The sign of {@code epsilon} is equivalent to performing the test against the polygon shrunk by
    * {@code Math.abs(epsilon)} if {@code epsilon < 0.0}, or against the polygon enlarged by
    * {@code epsilon} if {@code epsilon > 0.0}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code numberOfVertices == 0}, this method returns {@code false}.
    * <li>if {@code numberOfVertices == 1}, this method returns {@code false} if {@code epsilon < 0}
    * or if the query is at a distance from the polygon's only vertex that is greater than
    * {@code epsilon}, returns {@code true} otherwise.
    * <li>if {@code numberOfVertices == 2}, this method returns {@code false} if {@code epsilon < 0}
    * or if the query is at a distance from the polygon's only edge that is greater than
    * {@code epsilon}, returns {@code true} otherwise.
    * </ul>
    *
    * @param point the query. Not modified.
    * @param epsilon the tolerance to use during the test.
    * @return {@code true} if the query is considered to be inside the polygon, {@code false}
    *         otherwise.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default boolean isPointInside(Point2DReadOnly point, double epsilon)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.isPoint2DInsideConvexPolygon2D(point, getVertexBufferView(), getNumberOfVertices(), isClockwiseOrdered(),
                                                                       epsilon);
   }

   /**
    * Computes the coordinates of the closest point to the ray that belongs to this convex polygon.
    * <p>
    * WARNING: This methods assumes that the ray does not intersect with the polygon. Such scenario
    * should be handled with
    * {@link #intersectionWithRay(Line2DReadOnly, Point2DBasics, Point2DBasics)}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code false}.
    * <li>If the ray is parallel to the closest edge, the closest point to the ray origin is chosen.
    * </ul>
    * </p>
    *
    * @param ray the ray to find the closest point to. Not modified.
    * @param closestPointToPack the point in which the coordinates of the closest point are stored.
    *           Modified.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default boolean getClosestPointWithRay(Line2DReadOnly ray, Point2DBasics closestPointToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestPointToNonInterectingRay2D(ray.getPoint(), ray.getDirection(), getVertexBufferView(),
                                                                          getNumberOfVertices(), isClockwiseOrdered(), closestPointToPack);
   }

   /**
    * Computes the coordinates of the closest point to the ray that belongs to this convex polygon.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * WARNING: This methods assumes that the ray does not intersect with the polygon. Such scenario
    * should be handled with
    * {@link #intersectionWithRay(Line2DReadOnly, Point2DBasics, Point2DBasics)}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code null}.
    * <li>If the ray is parallel to the closest edge, the closest point to the ray origin is chosen.
    * </ul>
    * </p>
    *
    * @param ray the ray to find the closest point to. Not modified.
    * @return the coordinates of the closest point if the method succeeds, {@code null} otherwise.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default Point2DBasics getClosestPointWithRay(Line2DReadOnly ray)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestPointToNonInterectingRay2D(ray.getPoint(), ray.getDirection(), getVertexBufferView(),
                                                                          getNumberOfVertices(), isClockwiseOrdered());
   }

   /**
    * Calculates the minimum distance between the point and this polygon.
    * <p>
    * Note that if the point is inside this polygon, this method returns 0.0.
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @return the value of the distance between the point and this polygon.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default double distance(Point2DReadOnly point)
   {
      return Math.max(0.0, signedDistance(point));
   }

   /**
    * Returns minimum distance between the point and this polygon.
    * <p>
    * The returned value is negative if the point is inside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@link Double#NaN}.
    * <li>If the polygon has exactly one vertex, the returned value is positive and is equal to the
    * distance between the query and the polygon's vertex.
    * <li>If the polygon has exactly two vertices, the returned value is positive and is equal to
    * the distance and the line segment defined by the polygon's two vertices.
    * </ul>
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @return the distance between the query and the polygon, it is negative if the point is inside
    *         the polygon.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default double signedDistance(Point2DReadOnly point)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.signedDistanceFromPoint2DToConvexPolygon2D(point, getVertexBufferView(), getNumberOfVertices(),
                                                                                   isClockwiseOrdered());
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D convex polygon.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code false}.
    * <li>If the polygon has exactly one vertex, the result is the polygon only vertex, this method
    * returns {@code true}.
    * <li>If the query is inside the polygon, the method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to project on this polygon. Modified.
    * @return whether the method succeeded or not.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default boolean orthogonalProjection(Point2DBasics pointToProject)
   {
      return orthogonalProjection(pointToProject, pointToProject);
   }

   /**
    * Computes the orthogonal projection of a 2D point this 2D convex polygon.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code false}.
    * <li>If the polygon has exactly one vertex, the result is the polygon only vertex, this method
    * returns {@code true}.
    * <li>If the query is inside the polygon, the method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the coordinate of the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the convex polygon is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default boolean orthogonalProjection(Point2DReadOnly pointToProject, Point2DBasics projectionToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.orthogonalProjectionOnConvexPolygon2D(pointToProject, getVertexBufferView(), getNumberOfVertices(),
                                                                              isClockwiseOrdered(), projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D convex polygon.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code null}.
    * <li>If the polygon has exactly one vertex, the result is the polygon only vertex.
    * <li>If the query is inside the polygon, the method fails and returns {@code null}.
    * </ul>
    * </p>
    *
    * @param pointToProject the coordinate of the point to compute the projection of. Not modified.
    * @return the coordinates of the projection, or {@code null} if the method failed.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default Point2DBasics orthogonalProjectionCopy(Point2DReadOnly pointToProject)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.orthogonalProjectionOnConvexPolygon2D(pointToProject, getVertexBufferView(), getNumberOfVertices(),
                                                                              isClockwiseOrdered());
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the index of the
    * first vertex that is in the line-of-sight.
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code -1}.
    * <li>The observer is inside the polygon, this method fails and returns {@code -1}.
    * <li>The polygon has exactly one vertex, this method returns {@code 0} if the observer is
    * different from the polygon's vertex, or returns {@code -1} if the observer is equal to the
    * polygon's vertex.
    * </ul>
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the index of the first vertex that is in the line-of-sight, {@code -1} if this method
    *         fails.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default int lineOfSightStartIndex(Point2DReadOnly observer)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.lineOfSightStartIndex(observer, getVertexBufferView(), getNumberOfVertices(), isClockwiseOrdered());
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the index of the
    * last vertex that is in the line-of-sight.
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code -1}.
    * <li>The observer is inside the polygon, this method fails and returns {@code -1}.
    * <li>The polygon has exactly one vertex, this method returns {@code 0} if the observer is
    * different from the polygon's vertex, or returns {@code -1} if the observer is equal to the
    * polygon's vertex.
    * </ul>
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the index of the last vertex that is in the line-of-sight, {@code -1} if this method
    *         fails.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default int lineOfSightEndIndex(Point2DReadOnly observer)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.lineOfSightEndIndex(observer, getVertexBufferView(), getNumberOfVertices(), isClockwiseOrdered());
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the indices of the
    * first and last vertices that are in the line-of-sight.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code -1}.
    * <li>The observer is inside the polygon, this method fails and returns {@code -1}.
    * <li>The polygon has exactly one vertex, this method returns {@code 0} if the observer is
    * different from the polygon's vertex, or returns {@code -1} if the observer is equal to the
    * polygon's vertex.
    * </ul>
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the indices in order of the first and last vertices that are in the line-of-sight,
    *         {@code null} if this method fails.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default int[] lineOfSightIndices(Point2DReadOnly observer)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.lineOfSightIndices(observer, getVertexBufferView(), getNumberOfVertices(), isClockwiseOrdered());
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the first vertex
    * that is in the line-of-sight.
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code false}.
    * <li>The observer is inside the polygon, this method fails and returns {@code false}.
    * <li>The polygon has exactly one vertex, this method succeeds and packs the vertex coordinates
    * if the observer is different from the polygon's vertex, or returns {@code false} if the
    * observer is equal to the polygon's vertex.
    * </ul>
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param startVertexToPack point in which the coordinates of the first vertex in the
    *           line-of-sight are stored. Modified.
    * @return whether the method succeeded or not.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default boolean lineOfSightStartVertex(Point2DReadOnly observer, Point2DBasics startVertexToPack)
   {
      int lineOfSightStartIndex = lineOfSightStartIndex(observer);

      if (lineOfSightStartIndex == -1)
         return false;

      startVertexToPack.set(getVertex(lineOfSightStartIndex));

      return true;
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the last vertex
    * that is in the line-of-sight.
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code false}.
    * <li>The observer is inside the polygon, this method fails and returns {@code false}.
    * <li>The polygon has exactly one vertex, this method succeeds and packs the vertex coordinates
    * if the observer is different from the polygon's vertex, or returns {@code false} if the
    * observer is equal to the polygon's vertex.
    * </ul>
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param endVertexToPack point in which the coordinates of the last vertex in the line-of-sight
    *           are stored. Modified.
    * @return whether the method succeeded or not.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default boolean lineOfSightEndVertex(Point2DReadOnly observer, Point2DBasics endVertexToPack)
   {
      checkIfUpToDate();
      int lineOfSightEndIndex = lineOfSightEndIndex(observer);

      if (lineOfSightEndIndex == -1)
         return false;

      endVertexToPack.set(getVertex(lineOfSightEndIndex));

      return true;
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the first vertex
    * that is in the line-of-sight.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code null}.
    * <li>The observer is inside the polygon, this method fails and returns {@code null}.
    * <li>The polygon has exactly one vertex, this method succeeds and returns the vertex
    * coordinates if the observer is different from the polygon's vertex, or returns {@code null} if
    * the observer is equal to the polygon's vertex.
    * </ul>
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the coordinates of the first vertex in the line-of-sight or {@code null} if this
    *         method failed. Modified.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default Point2DBasics lineOfSightStartVertexCopy(Point2DReadOnly observer)
   {
      Point2D startVertex = new Point2D();
      boolean success = lineOfSightStartVertex(observer, startVertex);
      return success ? startVertex : null;
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the last vertex
    * that is in the line-of-sight.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code null}.
    * <li>The observer is inside the polygon, this method fails and returns {@code null}.
    * <li>The polygon has exactly one vertex, this method succeeds and returns the vertex
    * coordinates if the observer is different from the polygon's vertex, or returns {@code null} if
    * the observer is equal to the polygon's vertex.
    * </ul>
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the coordinates of the last vertex in the line-of-sight or {@code null} if this method
    *         failed. Modified.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default Point2DBasics lineOfSightEndVertexCopy(Point2DReadOnly observer)
   {
      Point2D endVertex = new Point2D();
      boolean success = lineOfSightEndVertex(observer, endVertex);
      return success ? endVertex : null;
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the first and last
    * vertices that is in the line-of-sight.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code null}.
    * <li>The observer is inside the polygon, this method fails and returns {@code null}.
    * <li>The polygon has exactly one vertex, this method succeeds and returns the vertex
    * coordinates if the observer is different from the polygon's vertex, or returns {@code null} if
    * the observer is equal to the polygon's vertex.
    * </ul>
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the coordinates in order of the first and last vertices in the line-of-sight or
    *         {@code null} if this method failed. Modified.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default Point2DBasics[] lineOfSightVertices(Point2DReadOnly observer)
   {
      Point2DBasics startVertex = lineOfSightStartVertexCopy(observer);
      Point2DBasics endVertex = lineOfSightEndVertexCopy(observer);
      if (startVertex == null || endVertex == null)
         return null;
      else
         return new Point2DBasics[] {startVertex, endVertex};
   }

   /**
    * Determines whether an observer can see the outside of the given edge of this convex polygon.
    * <p>
    * The edge is defined by its start {@code this.getVertex(edgeIndex)} and its end
    * {@code this.getNextVertex(edgeIndex)}.
    * </p>
    *
    * @param edgeIndex the vertex index of the start of the edge.
    * @param observer the coordinates of the observer. Not modified.
    * @return {@code true} if the observer can see the outside of the edge, {@code false} if the
    *         observer cannot see the outside or is lying on the edge.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default boolean canObserverSeeEdge(int edgeIndex, Point2DReadOnly observer)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.canObserverSeeEdge(edgeIndex, observer, getVertexBufferView(), getNumberOfVertices(), isClockwiseOrdered());
   }

   /**
    * Tests if the given point lies on an edge of this convex polygon.
    *
    * @param point the coordinates of the query. Not modified.
    * @return {@code true} if the point is considered to be on an edge of this polygon,
    *         {@code false} otherwise.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default boolean pointIsOnPerimeter(Point2DReadOnly point)
   {
      return Math.abs(signedDistance(point)) < 1.0E-10;
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and this
    * convex polygon 2D.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections.
    * <li>If no intersections exist, this method returns {@code 0} and the two intersection-to-pack
    * arguments remain unmodified.
    * <li>If there is only one intersection, this method returns {@code 1} and the coordinates of
    * the only intersection are stored in {@code firstIntersectionToPack}.
    * {@code secondIntersectionToPack} remains unmodified.
    * </ul>
    * </p>
    *
    * @param line the line that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default int intersectionWith(Line2DReadOnly line, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLine2DAndConvexPolygon2D(line.getPoint(), line.getDirection(), getVertexBufferView(),
                                                                                    getNumberOfVertices(), isClockwiseOrdered(), firstIntersectionToPack,
                                                                                    secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and this
    * convex polygon 2D.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections and
    * returns {@code null}.
    * <li>If no intersections exist, this method returns {@code null}.
    * </ul>
    * </p>
    *
    * @param line the line that may intersect this polygon. Not modified.
    * @return the coordinates of the intersections between the line and the polygon, or {@code null}
    *         if they do not intersect.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default Point2DBasics[] intersectionWith(Line2DReadOnly line)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLine2DAndConvexPolygon2D(line.getPoint(), line.getDirection(), getVertexBufferView(),
                                                                                    getNumberOfVertices(), isClockwiseOrdered());
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and this
    * convex polygon 2D.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections.
    * <li>If no intersections exist, this method returns {@code 0} and the two intersection-to-pack
    * arguments might be modified.
    * <li>If there is only one intersection, this method returns {@code 1} and the coordinates of
    * the only intersection are stored in {@code firstIntersectionToPack}.
    * {@code secondIntersectionToPack} might be modified.
    * </ul>
    * </p>
    *
    * @param ray the ray that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the ray and the polygon.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default int intersectionWithRay(Line2DReadOnly ray, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenRay2DAndConvexPolygon2D(ray.getPoint(), ray.getDirection(), getVertexBufferView(),
                                                                                   getNumberOfVertices(), isClockwiseOrdered(), firstIntersectionToPack,
                                                                                   secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and this
    * convex polygon 2D.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections and
    * returns {@code null}.
    * <li>If no intersections exist, this method returns {@code null}.
    * </ul>
    * </p>
    *
    * @param ray the ray that may intersect this polygon. Not modified.
    * @return the intersections between the ray and the polygon.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default Point2DBasics[] intersectionWithRay(Line2DReadOnly ray)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenRay2DAndConvexPolygon2D(ray.getPoint(), ray.getDirection(), getVertexBufferView(),
                                                                                   getNumberOfVertices(), isClockwiseOrdered());
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and
    * this convex polygon 2D.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections.
    * <li>If no intersections exist, this method returns {@code 0} and the two intersection-to-pack
    * arguments remain unmodified.
    * <li>If there is only one intersection, this method returns {@code 1} and the coordinates of
    * the only intersection are stored in {@code firstIntersectionToPack}.
    * {@code secondIntersectionToPack} remains unmodified.
    * <li>If the line segment is collinear to an edge:
    * <ul>
    * <li>The edge entirely contains the line segment: this method finds two intersections which are
    * the endpoints of the line segment.
    * <li>The line segment entirely contains the edge: this method finds two intersections which are
    * the vertices of the edge.
    * <li>The edge and the line segment partially overlap: this method finds two intersections which
    * the polygon's vertex that on the line segment and the line segment's endpoint that is on the
    * polygon's edge.
    * </ul>
    * </ul>
    * </p>
    *
    * @param lineSegment2D the line segment that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default int intersectionWith(LineSegment2DReadOnly lineSegment2D, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLineSegment2DAndConvexPolygon2D(lineSegment2D.getFirstEndpoint(), lineSegment2D.getSecondEndpoint(),
                                                                                           getVertexBufferView(), getNumberOfVertices(),
                                                                                           isClockwiseOrdered(), firstIntersectionToPack,
                                                                                           secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and
    * this convex polygon 2D.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections and
    * this method returns {@code null}.
    * <li>If no intersections exist, this method returns {@code null}.
    * <li>If the line segment is collinear to an edge:
    * <ul>
    * <li>The edge entirely contains the line segment: this method finds two intersections which are
    * the endpoints of the line segment.
    * <li>The line segment entirely contains the edge: this method finds two intersections which are
    * the vertices of the edge.
    * <li>The edge and the line segment partially overlap: this method finds two intersections which
    * the polygon's vertex that on the line segment and the line segment's endpoint that is on the
    * polygon's edge.
    * </ul>
    * </ul>
    * </p>
    *
    * @param lineSegment2D the line segment that may intersect this polygon. Not modified.
    * @return the intersections between the line segment and the polygon.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default Point2DBasics[] intersectionWith(LineSegment2DReadOnly lineSegment2D)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLineSegment2DAndConvexPolygon2D(lineSegment2D.getFirstEndpoint(), lineSegment2D.getSecondEndpoint(),
                                                                                           getVertexBufferView(), getNumberOfVertices(),
                                                                                           isClockwiseOrdered());
   }

   /**
    * Finds the index of the closest edge to the query.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has one or no vertices, this method fails and returns {@code -1}.
    * </ul>
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @return the index of the closest edge to the query.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default int getClosestEdgeIndex(Point2DReadOnly point)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestEdgeIndexToPoint2D(point, getVertexBufferView(), getNumberOfVertices(), isClockwiseOrdered());
   }

   /**
    * Finds the index of the closest edge to the query.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has one or no vertices, this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @param closestEdgeToPack the line segment used to store the result. Not modified.
    * @return whether this method succeeded or not.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default boolean getClosestEdge(Point2DReadOnly point, LineSegment2DBasics closestEdgeToPack)
   {
      int edgeIndex = getClosestEdgeIndex(point);
      if (edgeIndex == -1)
         return false;
      getEdge(edgeIndex, closestEdgeToPack);
      return true;
   }

   /**
    * Finds the index of the closest edge to the query.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has one or no vertices, this method fails and returns {@code null}.
    * </ul>
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @return the line segment representing the closest edge or {@code null} if this method failed.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default LineSegment2DBasics getClosestEdgeCopy(Point2DReadOnly point)
   {
      LineSegment2D closestEdge = new LineSegment2D();
      if (getClosestEdge(point, closestEdge))
         return closestEdge;
      else
         return null;
   }

   /**
    * Finds the index of the closest vertex to the query.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code -1}.
    * </ul>
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @return the index of the closest vertex to the query.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default int getClosestVertexIndex(Point2DReadOnly point)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestVertexIndexToPoint2D(point, getVertexBufferView(), getNumberOfVertices());
   }

   /**
    * Finds the index of the closest vertex to the query.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @param vertexToPack point used to store the result. Modified.
    * @return whether this method succeeded or not.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default boolean getClosestVertex(Point2DReadOnly point, Point2DBasics vertexToPack)
   {
      int vertexIndex = getClosestVertexIndex(point);
      if (vertexIndex == -1)
         return false;
      vertexToPack.set(getVertex(vertexIndex));
      return true;
   }

   /**
    * Finds the index of the closest vertex to the query.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code null}.
    * </ul>
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @return the coordinates of the closest vertex, or {@code null} if this method failed.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default Point2DBasics getClosestVertexCopy(Point2DReadOnly point)
   {
      int vertexIndex = getClosestVertexIndex(point);
      if (vertexIndex == -1)
         return null;
      return new Point2D(getVertex(vertexIndex));
   }

   /**
    * Finds the index of the closest vertex to the given line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code -1}.
    * </ul>
    * </p>
    *
    * @param line the query. Not modified.
    * @return the index of the closest vertex to the query.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default int getClosestVertexIndex(Line2DReadOnly line)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestVertexIndexToLine2D(line.getPoint(), line.getDirection(), getVertexBufferView(), getNumberOfVertices());
   }

   /**
    * Finds the index of the closest vertex to the given line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param line the query. Not modified.
    * @param vertexToPack point used to store the result. Modified.
    * @return whether this method succeeded or not.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default boolean getClosestVertex(Line2DReadOnly line, Point2DBasics vertexToPack)
   {
      int vertexIndex = getClosestVertexIndex(line);
      if (vertexIndex == -1)
         return false;
      vertexToPack.set(getVertex(vertexIndex));
      return true;
   }

   /**
    * Finds the index of the closest vertex to the given line.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param line the query. Not modified.
    * @return the coordinates of the closest vertex or {@code null} if this method failed.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    */
   default Point2DBasics getClosestVertexCopy(Line2DReadOnly line)
   {
      int vertexIndex = getClosestVertexIndex(line);
      if (vertexIndex == -1)
         return null;
      return new Point2D(getVertex(vertexIndex));
   }

   /**
    * Packs the endpoints of an edge of this polygon into {@code edgeToPack}.
    *
    * @param edgeIndex index of the vertex that starts the edge.
    * @param edgeToPack line segment used to store the edge endpoints. Modified.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default void getEdge(int edgeIndex, LineSegment2DBasics edgeToPack)
   {
      edgeToPack.set(getVertex(edgeIndex), getNextVertex(edgeIndex));
   }

   /**
    * Copies this polygon, translates the copy, and returns it.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param translation the translation to apply to the copy of this polygon. Not modified.
    * @return the copy of this polygon translated.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2DBasics#update()} has not been called
    *            since last time this polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default ConvexPolygon2DBasics translateCopy(Tuple2DReadOnly translation)
   {
      ConvexPolygon2D copy = new ConvexPolygon2D(this);
      copy.translate(translation);
      return copy;
   }

   /**
    * Tests on a per vertex and per component basis, if this polygon is exactly equal to
    * {@code other}.
    *
    * @param other the other polygon to compare against this. Not modified.
    * @return {@code true} if the two polygons are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(ConvexPolygon2DReadOnly other)
   {
      if (other == null)
         return false;

      if (getNumberOfVertices() != other.getNumberOfVertices())
         return false;

      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         Point2DReadOnly thisVertex = getVertexBufferView().get(i);
         Point2DReadOnly otherVertex = other.getVertexBufferView().get(i);
         if (!thisVertex.equals(otherVertex))
            return false;
      }
      return true;
   }

   /**
    * Tests on a per-component basis on every vertices if this convex polygon is equal to
    * {@code other} with the tolerance {@code epsilon}.
    * <p>
    * The method returns {@code false} if the two polygons have different size.
    * </p>
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two line segments are equal, {@code false} otherwise.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default boolean epsilonEquals(ConvexPolygon2DReadOnly other, double epsilon)
   {
      if (getNumberOfVertices() != other.getNumberOfVertices())
         return false;

      for (int i = 0; i < other.getNumberOfVertices(); i++)
      {
         Point2DReadOnly thisVertex = getVertexBufferView().get(i);
         Point2DReadOnly otherVertex = other.getVertexBufferView().get(i);
         if (!thisVertex.epsilonEquals(otherVertex, epsilon))
            return false;
      }

      return true;
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two convex polygons are
    * geometrically similar.
    * <p>
    * This method performs the comparison on a per vertex basis while accounting for a possible
    * shift in the polygon indexing. For instance, two polygons that have the same vertices in
    * clockwise or counter-clockwise order, are considered geometrically equal even if they do not
    * start with the same vertex.
    * </p>
    *
    * @param other the convex polygon to compare to.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the convex polygons represent the same geometry, {@code false}
    *         otherwise.
    */
   default boolean geometricallyEquals(ConvexPolygon2DReadOnly other, double epsilon)
   {
      if (getNumberOfVertices() != other.getNumberOfVertices())
         return false;

      boolean sameClockwiseDirection = isClockwiseOrdered() == other.isClockwiseOrdered();

      int indexOfClosestOtherPoint = other.getClosestVertexIndex(getVertex(0));

      for (int thisPointIndex = 0, otherPointIndex; thisPointIndex < getNumberOfVertices(); thisPointIndex++)
      {
         if (sameClockwiseDirection)
            otherPointIndex = (indexOfClosestOtherPoint + thisPointIndex) % getNumberOfVertices();
         else
            otherPointIndex = (getNumberOfVertices() + indexOfClosestOtherPoint - thisPointIndex) % getNumberOfVertices();

         Point2DReadOnly thisVertex = getVertexBufferView().get(thisPointIndex);
         Point2DReadOnly otherVertex = other.getVertexBufferView().get(otherPointIndex);
         if (!thisVertex.geometricallyEquals(otherVertex, epsilon))
            return false;
      }

      return true;
   }
}
