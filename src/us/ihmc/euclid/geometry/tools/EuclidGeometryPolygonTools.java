package us.ihmc.euclid.geometry.tools;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.ONE_TEN_MILLIONTH;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.ONE_TRILLIONTH;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.areVector2DsParallel;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.distanceBetweenPoint2Ds;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.distanceFromPoint2DToLine2D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.distanceFromPoint2DToLineSegment2D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.distanceFromPoint2DToRay2D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.distanceSquaredBetweenPoint2Ds;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.doLine2DAndLineSegment2DIntersect;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.isPoint2DInFrontOfRay2D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.isPoint2DOnLine2D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.isPoint2DOnSideOfLine2D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.orthogonalProjectionOnLineSegment2D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.percentageOfIntersectionBetweenTwoLine2Ds;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.perpendicularVector2D;

import java.util.Collections;
import java.util.List;
import java.util.Random;

import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

/**
 * This class provides a variety of tools to perform operations with polygons.
 *
 * @author Sylvain Bertrand
 */
public class EuclidGeometryPolygonTools
{
   private static final Random random = new Random();
   static final double EPSILON = 1.0e-7;

   /**
    * Human readable enum that helps defining search criteria for some of the search methods in this
    * tool class.
    *
    * @author Sylvain Bertrand
    */
   public static enum Bound
   {
      /**
       * Refers to lower values, i.e. towards -&infin;.
       */
      MIN
      {
         @Override
         boolean isFirstBetter(double first, double second)
         {
            return first < second;
         }
      },
      /**
       * Refers to higher values, i.e. towards +&infin;.
       */
      MAX
      {
         @Override
         boolean isFirstBetter(double first, double second)
         {
            return first > second;
         }
      };

      abstract boolean isFirstBetter(double first, double second);
   };

   /**
    * Tests if the polygon defined by the given {@code vertices} is convex at the vertex defined by
    * the given {@code vertexIndex}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>the method returns {@code false} if the vertex, next and previous vertices lie on the same
    * line.
    * <li>the method returns {@code false} if the polygon has less than 3 vertices.
    * </ul>
    * </p>
    *
    * @param vertexIndex the index of the vertex to be tested for convexity.
    * @param vertices the list of vertices defining the polygon to test. Not modified.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return {@code true} if the polygon is convex at the given vertex, {@code false} otherwise.
    * @throws IndexOutOfBoundsException if {@code vertexIndex} is either negative or greater or
    *            equal than {@code numberOfVertices}.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static boolean isPolygon2DConvexAtVertex(int vertexIndex, List<? extends Point2DReadOnly> vertices, boolean clockwiseOrdered)
   {
      return isPolygon2DConvexAtVertex(vertexIndex, vertices, vertices.size(), clockwiseOrdered);
   }

   /**
    * Tests if the polygon defined by the given {@code vertices} is convex at the vertex defined by
    * the given {@code vertexIndex}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>the method returns {@code false} if the vertex, next and previous vertices lie on the same
    * line.
    * <li>the method returns {@code false} if the polygon has less than 3 vertices.
    * </ul>
    * </p>
    *
    * @param vertexIndex the index of the vertex to be tested for convexity.
    * @param vertices the list of vertices defining the polygon to test. Not modified.
    * @param numberOfVertices the number of vertices relevant to the polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return {@code true} if the polygon is convex at the given vertex, {@code false} otherwise.
    * @throws IndexOutOfBoundsException if {@code vertexIndex} is either negative or greater or
    *            equal than {@code numberOfVertices}.
    * @throws IndexOutOfBoundsException if {@code vertexIndex} is either negative or greater or
    *            equal than {@code numberOfVertices}.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static boolean isPolygon2DConvexAtVertex(int vertexIndex, List<? extends Point2DReadOnly> vertices, int numberOfVertices, boolean clockwiseOrdered)
   {
      checkNumberOfVertices(vertices, numberOfVertices);
      Point2DReadOnly vertex = vertices.get(vertexIndex);
      Point2DReadOnly previousVertex = vertices.get(previous(vertexIndex, numberOfVertices));
      Point2DReadOnly nextVertex = vertices.get(next(vertexIndex, numberOfVertices));

      return isPoint2DOnSideOfLine2D(vertex, previousVertex, nextVertex, clockwiseOrdered);
   }

   /**
    * In-place and garbage free implementation of the
    * <a href="https://en.wikipedia.org/wiki/Gift_wrapping_algorithm">Gift wrapping algorithm</a>
    * for computing the convex hull 2D of a set of points.
    * <p>
    * The given list {@code vertices} is reordered such that the vertices of the clockwise convex
    * hull are positioned first. The method returns the number of vertices that compose the convex
    * hull.
    * </p>
    *
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Modified.
    * @return the size of the convex hull.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int inPlaceGiftWrapConvexHull2D(List<? extends Point2DReadOnly> vertices)
   {
      return inPlaceGiftWrapConvexHull2D(vertices, vertices.size());
   }

   /**
    * In-place and garbage free implementation of the
    * <a href="https://en.wikipedia.org/wiki/Gift_wrapping_algorithm">Gift wrapping algorithm</a>
    * for computing the convex hull 2D of a set of points.
    * <p>
    * The given list {@code vertices} is reordered such that the vertices of the clockwise convex
    * hull are positioned first. The method returns the number of vertices that compose the convex
    * hull.
    * </p>
    *
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Modified.
    * @param numberOfVertices specifies the number of relevant points in the list. The algorithm
    *           will only process the points &in; [0; {@code numberOfVertices}[.
    * @return the size of the convex hull.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int inPlaceGiftWrapConvexHull2D(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return 0;

      checkNumberOfVertices(vertices, numberOfVertices);

      /*
       * Set the first vertex to be the one with the lowest x-coordinate. If the lowest x-coordinate
       * exists in more than one vertex in the list, the vertex with the highest y-coordinate out of
       * the candidates is chosen.
       */
      Collections.swap(vertices, findMinXMaxYVertexIndex(vertices, numberOfVertices), 0);
      Point2DReadOnly firstVertex = vertices.get(0);

      // This is the last index that belongs to the convex polygon that has been found so far.
      int lastHullVertexIndex = 0;

      while (lastHullVertexIndex < numberOfVertices - 1)
      {
         Point2DReadOnly lastHullVertex = vertices.get(lastHullVertexIndex);

         // Index to keep track of the potential next vertex of the polygon.
         int candidateIndex = lastHullVertexIndex + 1;
         Point2DReadOnly candidateVertex = vertices.get(candidateIndex);

         while (candidateVertex.epsilonEquals(lastHullVertex, EPSILON))
         { // Remove any duplicate vertices
            Collections.swap(vertices, candidateIndex, --numberOfVertices);
            candidateVertex = vertices.get(candidateIndex);

            if (numberOfVertices == 1)
               return numberOfVertices;
         }

         for (int vertexIndex = lastHullVertexIndex + 2; vertexIndex <= numberOfVertices; vertexIndex++)
         {
            int wrappedIndex = wrap(vertexIndex, numberOfVertices);
            Point2DReadOnly vertex = vertices.get(wrappedIndex);

            if (vertex.epsilonEquals(candidateVertex, EPSILON) || wrappedIndex != 0 && vertex.epsilonEquals(firstVertex, EPSILON))
            { // Remove duplicate vertices
               Collections.swap(vertices, wrappedIndex, --numberOfVertices);
               vertex = vertices.get(wrappedIndex);
            }

            if (isPoint2DOnLeftSideOfLine2D(vertex, lastHullVertex, candidateVertex))
            { // vertex is located outside => candidateVertex is not the next polygon vertex, vertex might be though.
               candidateIndex = wrappedIndex;
               candidateVertex = vertex;
            }
         }

         if (candidateIndex == 0)
         {
            /*
             * Got back to the first vertex of the polygon: we're done and the size of the polygon
             * is defined by the index of the last vertex found.
             */
            return lastHullVertexIndex + 1;
         }
         else
         {
            /*
             * Swap the candidate to be located right after the last vertex found in the list.
             */
            lastHullVertexIndex++;
            Collections.swap(vertices, lastHullVertexIndex, candidateIndex);
         }
      }

      return numberOfVertices;
   }

   /**
    * In-place and garbage free implementation of the
    * <a href="https://en.wikipedia.org/wiki/Graham_scan">Graham scan algorithm</a> for computing
    * the convex hull 2D of a set of points.
    * <p>
    * The given list {@code vertices} is reordered such that the vertices of the clockwise convex
    * hull are positioned first. The method returns the number of vertices that compose the convex
    * hull.
    * </p>
    *
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Modified.
    * @return the size of the convex hull.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int inPlaceGrahamScanConvexHull2D(List<? extends Point2DReadOnly> vertices)
   {
      return inPlaceGrahamScanConvexHull2D(vertices, vertices.size());
   }

   /**
    * In-place and garbage free implementation of the
    * <a href="https://en.wikipedia.org/wiki/Graham_scan">Graham scan algorithm</a> for computing
    * the convex hull 2D of a set of points.
    * <p>
    * The given list {@code vertices} is reordered such that the vertices of the clockwise convex
    * hull are positioned first. The method returns the number of vertices that compose the convex
    * hull.
    * </p>
    *
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Modified.
    * @param numberOfVertices specifies the number of relevant points in the list. The algorithm
    *           will only process the points &in; [0; {@code numberOfVertices}[.
    * @return the size of the convex hull.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int inPlaceGrahamScanConvexHull2D(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return 0;

      checkNumberOfVertices(vertices, numberOfVertices);

      grahamScanAngleSort(vertices, numberOfVertices);

      if (numberOfVertices <= 3)
      {
         boolean areAllVerticesEqual = true;
         Point2DReadOnly firstVertex = vertices.get(0);

         for (int vertexIndex = 1; vertexIndex < numberOfVertices; vertexIndex++)
         {
            if (!firstVertex.epsilonEquals(vertices.get(vertexIndex), EPSILON))
            {
               areAllVerticesEqual = false;
               break;
            }
         }

         return areAllVerticesEqual ? 1 : numberOfVertices;
      }

      int currentIndex = 1;

      while (currentIndex < numberOfVertices)
      {
         if (isPolygon2DConvexAtVertex(currentIndex, vertices, numberOfVertices, true))
         { // Convex at the current vertex: move on to next.
            currentIndex++;
         }
         else
         { // Not convex: remove the current vertex.
            moveElementToEnd(vertices, currentIndex, numberOfVertices);
            numberOfVertices--; // The vertex is not part of the convex hull.

            if (numberOfVertices == 1)
               return numberOfVertices;

            /*
             * Need verify the previous vertex since the current has been updated but always knowing
             * that the first vertex has to be part of the hull.
             */
            currentIndex = Math.max(0, currentIndex - 1);
         }
      }

      return numberOfVertices;
   }

   /**
    * Computes the area and centroid (optional) of a convex polygon defined by its size
    * {@code numberOfVertices} and vertices {@code convexPolygon2D}.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code numberOfVertices == 0}, this method returns {@link Double#NaN} and
    * {@code centroidToPack} is set to {@link Double#NaN}.
    * <li>if {@code numberOfVertices < 3}, this method returns {@code 0.0} and
    * {@code centroidToPack} is set to average of the polygon vertices.
    * </ul>
    * </p>
    *
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @param centroidToPack point 2D in which the centroid of the convex polygon is stored. Can be
    *           {@code null}. Modified.
    * @return the area of the convex polygon.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static double computeConvexPolyong2DArea(List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices, boolean clockwiseOrdered,
                                                   Point2DBasics centroidToPack)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);

      if (numberOfVertices == 0)
      {
         if (centroidToPack != null)
            centroidToPack.setToNaN();
         return Double.NaN;
      }
      else if (numberOfVertices < 3)
      {
         if (centroidToPack != null)
         {
            centroidToPack.setToZero();
            for (int i = 0; i < numberOfVertices; i++)
               centroidToPack.add(convexPolygon2D.get(i));
            centroidToPack.scale(1.0 / numberOfVertices);
         }
         return 0.0;
      }
      else
      {
         double area = 0.0;
         double Cx = 0.0;
         double Cy = 0.0;

         if (clockwiseOrdered)
         {
            for (int i = 0; i < numberOfVertices; i++)
            {
               Point2DReadOnly ci = convexPolygon2D.get(i);
               Point2DReadOnly ciMinus1 = convexPolygon2D.get(previous(i, numberOfVertices));

               double weight = ci.getX() * ciMinus1.getY() - ciMinus1.getX() * ci.getY();

               Cx += (ci.getX() + ciMinus1.getX()) * weight;
               Cy += (ci.getY() + ciMinus1.getY()) * weight;

               area += weight;
            }
         }
         else
         {
            for (int i = 0; i < numberOfVertices; i++)
            {
               Point2DReadOnly ci = convexPolygon2D.get(i);
               Point2DReadOnly ciPlus1 = convexPolygon2D.get(next(i, numberOfVertices));

               double weight = ci.getX() * ciPlus1.getY() - ciPlus1.getX() * ci.getY();

               Cx += (ci.getX() + ciPlus1.getX()) * weight;
               Cy += (ci.getY() + ciPlus1.getY()) * weight;

               area += weight;
            }
         }

         area *= 0.5;

         if (centroidToPack != null)
         {
            if (area < 1.0e-5)
            {
               centroidToPack.set(convexPolygon2D.get(0));
            }
            else
            {
               centroidToPack.set(Cx, Cy);
               centroidToPack.scale(1.0 / (6.0 * area));
            }
         }

         return area;
      }
   }

   /**
    * Computes the vector that points to the outside and orthogonal to the
    * {@code edgeIndex}<sup>th</sup> edge of the given convex polygon.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    *
    * @param edgeIndex index of the vertex the edge starts from.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @param normalToPack the vector that is orthogonal to the edge and points toward the outside of
    *           the polygon. Modified.
    * @return whether the method succeeds or not.
    * @throws IndexOutOfBoundsException if {@code edgeIndex} is either negative or greater or equal
    *            than {@code numberOfVertices}.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static boolean edgeNormal(int edgeIndex, List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices, boolean clockwiseOrdered,
                                    Vector2DBasics normalToPack)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);

      if (numberOfVertices <= 1)
         return false;

      checkEdgeIndex(edgeIndex, numberOfVertices);

      Point2DReadOnly edgeStart = convexPolygon2D.get(edgeIndex);
      Point2DReadOnly edgeEnd = convexPolygon2D.get(next(edgeIndex, numberOfVertices));

      normalToPack.sub(edgeEnd, edgeStart);
      normalToPack.normalize();
      perpendicularVector2D(normalToPack, normalToPack);

      if (!clockwiseOrdered)
         normalToPack.negate();

      return true;
   }

   /**
    * Determines if the point is inside the convex polygon given the tolerance {@code epsilon}.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * It is equivalent to performing the test against the polygon shrunk by
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
    *
    * @param pointX the x-coordinate of the query.
    * @param pointY the y-coordinate of the query.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @param epsilon the tolerance to use during the test.
    * @return {@code true} if the query is considered to be inside the polygon, {@code false}
    *         otherwise.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static boolean isPoint2DInsideConvexPolygon2D(double pointX, double pointY, List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                                        boolean clockwiseOrdered, double epsilon)
   {
      return signedDistanceFromPoint2DToConvexPolygon2D(pointX, pointY, convexPolygon2D, numberOfVertices, clockwiseOrdered) <= epsilon;
   }

   /**
    * Determines if the point is inside the convex polygon given the tolerance {@code epsilon}.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * It is equivalent to performing the test against the polygon shrunk by
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
    *
    * @param point the coordinates of the query. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @param epsilon the tolerance to use during the test.
    * @return {@code true} if the query is considered to be inside the polygon, {@code false}
    *         otherwise.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static boolean isPoint2DInsideConvexPolygon2D(Point2DReadOnly point, List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                                        boolean clockwiseOrdered, double epsilon)
   {
      return isPoint2DInsideConvexPolygon2D(point.getX(), point.getY(), convexPolygon2D, numberOfVertices, clockwiseOrdered, epsilon);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and a given
    * convex polygon 2D.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
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
    * @param pointOnLine a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int intersectionBetweenLine2DAndConvexPolygon2D(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection,
                                                                 List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                                                 boolean clockwiseOrdered, Point2DBasics firstIntersectionToPack,
                                                                 Point2DBasics secondIntersectionToPack)
   {
      double pointOnLineX = pointOnLine.getX();
      double pointOnLineY = pointOnLine.getY();
      double lineDirectionX = lineDirection.getX();
      double lineDirectionY = lineDirection.getY();

      return intersectionBetweenLine2DAndConvexPolygon2D(pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, convexPolygon2D, numberOfVertices,
                                                         clockwiseOrdered, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and a given
    * convex polygon 2D.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
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
    * @param pointOnLineX the x-coordinate of a point on the line.
    * @param pointOnLineY the y-coordinate of a point on the line.
    * @param lineDirectionX the x-component of the direction of the line.
    * @param lineDirectionY the y-component of the direction of the line.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int intersectionBetweenLine2DAndConvexPolygon2D(double pointOnLineX, double pointOnLineY, double lineDirectionX, double lineDirectionY,
                                                                 List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                                                 boolean clockwiseOrdered, Point2DBasics firstIntersectionToPack,
                                                                 Point2DBasics secondIntersectionToPack)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);

      if (numberOfVertices == 0)
         return 0;

      if (numberOfVertices == 1)
      {
         Point2DReadOnly vertex = convexPolygon2D.get(0);
         if (isPoint2DOnLine2D(vertex.getX(), vertex.getY(), pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY))
         {
            firstIntersectionToPack.set(convexPolygon2D.get(0));
            return 1;
         }
         else
         {
            return 0;
         }
      }

      int firstEdgeIndex = nextEdgeIndexIntersectingWithLine2D(-1, pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, convexPolygon2D,
                                                               numberOfVertices);
      if (firstEdgeIndex < 0)
         return 0;

      Point2DReadOnly edgeStart = convexPolygon2D.get(firstEdgeIndex);
      Point2DReadOnly edgeEnd = convexPolygon2D.get(next(firstEdgeIndex, numberOfVertices));
      boolean success = intersectionBetweenLine2DAndLineSegment2D(pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, edgeStart.getX(),
                                                                  edgeStart.getY(), edgeEnd.getX(), edgeEnd.getY(), firstIntersectionToPack);
      if (!success)
         throw new RuntimeException("Inconsistency in algorithms.");

      int secondEdgeIndex = nextEdgeIndexIntersectingWithLine2D(firstEdgeIndex, pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, convexPolygon2D,
                                                                numberOfVertices);

      if (secondEdgeIndex < 0)
         return 1;

      edgeStart = convexPolygon2D.get(secondEdgeIndex);
      edgeEnd = convexPolygon2D.get(next(secondEdgeIndex, numberOfVertices));
      success = intersectionBetweenLine2DAndLineSegment2D(pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, edgeStart.getX(), edgeStart.getY(),
                                                          edgeEnd.getX(), edgeEnd.getY(), secondIntersectionToPack);
      if (!success)
         throw new RuntimeException("Inconsistency in algorithms.");

      if (!firstIntersectionToPack.epsilonEquals(secondIntersectionToPack, EPSILON))
         return 2;

      secondEdgeIndex = nextEdgeIndexIntersectingWithLine2D(secondEdgeIndex, pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, convexPolygon2D,
                                                            numberOfVertices);

      if (secondEdgeIndex < 0 || secondEdgeIndex == firstEdgeIndex)
         return 1;

      edgeStart = convexPolygon2D.get(secondEdgeIndex);
      edgeEnd = convexPolygon2D.get(next(secondEdgeIndex, numberOfVertices));
      success = intersectionBetweenLine2DAndLineSegment2D(pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, edgeStart.getX(), edgeStart.getY(),
                                                          edgeEnd.getX(), edgeEnd.getY(), secondIntersectionToPack);
      if (!success)
         throw new RuntimeException("Inconsistency in algorithms.");

      return 2;
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and a given
    * convex polygon 2D.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
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
    * @param pointOnLine a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return the intersections between between the line and the polygon or {@code null} if the
    *         method failed or if there is no intersections.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static Point2D[] intersectionBetweenLine2DAndConvexPolygon2D(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection,
                                                                       List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                                                       boolean clockwiseOrdered)
   {
      Point2D firstIntersection = new Point2D();
      Point2D secondIntersection = new Point2D();

      int numberOfIntersections = intersectionBetweenLine2DAndConvexPolygon2D(pointOnLine, lineDirection, convexPolygon2D, numberOfVertices, clockwiseOrdered,
                                                                              firstIntersection, secondIntersection);

      switch (numberOfIntersections)
      {
      case 1:
         return new Point2D[] {firstIntersection};
      case 2:
         return new Point2D[] {firstIntersection, secondIntersection};
      default:
         return null;
      }
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and a
    * given convex polygon 2D.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
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
    * @param lineSegmentStart the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd the second endpoint of the line segment. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line segment and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line segment and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line segment and the polygon.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int intersectionBetweenLineSegment2DAndConvexPolygon2D(Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd,
                                                                        List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                                                        boolean clockwiseOrdered, Point2DBasics firstIntersectionToPack,
                                                                        Point2DBasics secondIntersectionToPack)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);

      if (numberOfVertices == 0)
         return 0;

      if (numberOfVertices == 1)
      {
         Point2DReadOnly vertex = convexPolygon2D.get(0);
         if (distanceFromPoint2DToLineSegment2D(vertex, lineSegmentStart, lineSegmentEnd) < ONE_TRILLIONTH)
         {
            firstIntersectionToPack.set(convexPolygon2D.get(0));
            return 1;
         }
         else
         {
            return 0;
         }
      }

      double lineSegmentDx = lineSegmentEnd.getX() - lineSegmentStart.getX();
      double lineSegmentDy = lineSegmentEnd.getY() - lineSegmentStart.getY();

      int foundIntersections = 0;
      if (numberOfVertices == 2)
      {
         Point2DReadOnly vertex0 = convexPolygon2D.get(0);
         Point2DReadOnly vertex1 = convexPolygon2D.get(1);
         if (distanceFromPoint2DToLineSegment2D(vertex0, lineSegmentStart, lineSegmentEnd) < ONE_TRILLIONTH)
         {
            firstIntersectionToPack.set(vertex0);
            foundIntersections++;
         }

         if (distanceFromPoint2DToLineSegment2D(vertex1, lineSegmentStart, lineSegmentEnd) < ONE_TRILLIONTH)
         {
            if (foundIntersections == 0)
               firstIntersectionToPack.set(vertex1);
            else
               secondIntersectionToPack.set(vertex1);
            foundIntersections++;
         }

         if (foundIntersections == 2)
            return 2;
      }

      for (int edgeIndex = 0; edgeIndex < numberOfVertices; edgeIndex++)
      {
         Point2DReadOnly edgeStart = convexPolygon2D.get(edgeIndex);
         Point2DReadOnly edgeEnd = convexPolygon2D.get(next(edgeIndex, numberOfVertices));

         // check if the end points of the line segments are on this edge
         if (distanceFromPoint2DToLineSegment2D(lineSegmentStart, edgeStart, edgeEnd) < ONE_TRILLIONTH)
         {
            if (foundIntersections == 0)
               firstIntersectionToPack.set(lineSegmentStart);
            else
               secondIntersectionToPack.set(lineSegmentStart);

            foundIntersections++;
            if (foundIntersections == 2 && firstIntersectionToPack.epsilonEquals(secondIntersectionToPack, ONE_TRILLIONTH))
               foundIntersections--;
            if (foundIntersections == 2)
               return foundIntersections; // No intersection left to find.
         }
         if (distanceFromPoint2DToLineSegment2D(lineSegmentEnd, edgeStart, edgeEnd) < ONE_TRILLIONTH)
         {
            if (foundIntersections == 0)
               firstIntersectionToPack.set(lineSegmentEnd);
            else
               secondIntersectionToPack.set(lineSegmentEnd);

            foundIntersections++;
            if (foundIntersections == 2 && firstIntersectionToPack.epsilonEquals(secondIntersectionToPack, ONE_TRILLIONTH))
               foundIntersections--;
            if (foundIntersections == 2)
               return foundIntersections; // No intersection left to find.
         }

         double edgeVectorX = edgeEnd.getX() - edgeStart.getX();
         double edgeVectorY = edgeEnd.getY() - edgeStart.getY();
         double lambda = percentageOfIntersectionBetweenTwoLine2Ds(edgeStart.getX(), edgeStart.getY(), edgeVectorX, edgeVectorY, lineSegmentStart.getX(),
                                                                   lineSegmentStart.getY(), lineSegmentDx, lineSegmentDy);
         if (Double.isNaN(lambda))
            continue;

         // check if within edge bounds:
         if (lambda < 0.0 - ONE_TRILLIONTH || lambda > 1.0 + ONE_TRILLIONTH)
            continue;

         if (lambda < 0.0)
            lambda = 0.0;
         else if (lambda > 1.0)
            lambda = 1.0;

         double candidateX = edgeStart.getX() + lambda * edgeVectorX;
         double candidateY = edgeStart.getY() + lambda * edgeVectorY;

         // check if within segment bounds:
         if (!isPoint2DInFrontOfRay2D(candidateX, candidateY, lineSegmentStart.getX(), lineSegmentStart.getY(), lineSegmentDx, lineSegmentDy))
            continue;
         if (!isPoint2DInFrontOfRay2D(candidateX, candidateY, lineSegmentEnd.getX(), lineSegmentEnd.getY(), -lineSegmentDx, -lineSegmentDy))
            continue;

         if (foundIntersections == 0)
            firstIntersectionToPack.set(candidateX, candidateY);
         else
            secondIntersectionToPack.set(candidateX, candidateY);

         foundIntersections++;
         if (foundIntersections == 2 && firstIntersectionToPack.epsilonEquals(secondIntersectionToPack, ONE_TRILLIONTH))
            foundIntersections--;
         if (foundIntersections == 2)
            return foundIntersections; // No intersection left to find.
      }

      return foundIntersections;
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and a
    * given convex polygon 2D.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
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
    * @param lineSegmentStart the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd the second endpoint of the line segment. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return the intersections between the line segment and the polygon.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static Point2D[] intersectionBetweenLineSegment2DAndConvexPolygon2D(Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd,
                                                                              List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                                                              boolean clockwiseOrdered)
   {
      Point2D firstIntersection = new Point2D();
      Point2D secondIntersection = new Point2D();
      int numberOfIntersections = intersectionBetweenLineSegment2DAndConvexPolygon2D(lineSegmentStart, lineSegmentEnd, convexPolygon2D, numberOfVertices,
                                                                                     clockwiseOrdered, firstIntersection, secondIntersection);

      switch (numberOfIntersections)
      {
      case 1:
         return new Point2D[] {firstIntersection};
      case 2:
         return new Point2D[] {firstIntersection, secondIntersection};
      default:
         return null;
      }
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and a given
    * convex polygon 2D.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
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
    * @param rayOrigin the ray's origin. Not modified.
    * @param rayDirection the ray's direction. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the ray and the polygon.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int intersectionBetweenRay2DAndConvexPolygon2D(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection,
                                                                List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices, boolean clockwiseOrdered,
                                                                Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      int numberOfIntersections = intersectionBetweenLine2DAndConvexPolygon2D(rayOrigin, rayDirection, convexPolygon2D, numberOfVertices, clockwiseOrdered,
                                                                              firstIntersectionToPack, secondIntersectionToPack);

      if (numberOfIntersections == 2 && !isPoint2DInFrontOfRay2D(secondIntersectionToPack, rayOrigin, rayDirection))
      {
         numberOfIntersections--;
      }

      if (numberOfIntersections >= 1 && !isPoint2DInFrontOfRay2D(firstIntersectionToPack, rayOrigin, rayDirection))
      {
         numberOfIntersections--;
         firstIntersectionToPack.set(secondIntersectionToPack);
      }

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and a given
    * convex polygon 2D.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
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
    * @param rayOrigin the ray's origin. Not modified.
    * @param rayDirection the ray's direction. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return the intersections between the ray and the polygon.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static Point2D[] intersectionBetweenRay2DAndConvexPolygon2D(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection,
                                                                      List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                                                      boolean clockwiseOrdered)
   {
      Point2D firstIntersection = new Point2D();
      Point2D secondIntersection = new Point2D();
      int numberOfIntersections = intersectionBetweenRay2DAndConvexPolygon2D(rayOrigin, rayDirection, convexPolygon2D, numberOfVertices, clockwiseOrdered,
                                                                             firstIntersection, secondIntersection);

      switch (numberOfIntersections)
      {
      case 1:
         return new Point2D[] {firstIntersection};
      case 2:
         return new Point2D[] {firstIntersection, secondIntersection};
      default:
         return null;
      }
   }

   /**
    * Returns minimum distance between the point and the polygon.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * The return value is negative if the point is inside the polygon.
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
    * @param pointX the x-coordinate of the query.
    * @param pointY the y-coordinate of the query.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return the distance between the query and the polygon, it is negative if the point is inside
    *         the polygon.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static double signedDistanceFromPoint2DToConvexPolygon2D(double pointX, double pointY, List<? extends Point2DReadOnly> convexPolygon2D,
                                                                   int numberOfVertices, boolean clockwiseOrdered)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);

      if (numberOfVertices == 0)
         return Double.NaN;

      if (numberOfVertices == 1)
         return distanceBetweenPoint2Ds(pointX, pointY, convexPolygon2D.get(0));

      if (numberOfVertices == 2)
         return distanceFromPoint2DToLineSegment2D(pointX, pointY, convexPolygon2D.get(0), convexPolygon2D.get(1));

      boolean isQueryOutsidePolygon = false;
      double minDistance = Double.POSITIVE_INFINITY;

      for (int index = 0; index < numberOfVertices; index++)
      {
         Point2DReadOnly edgeStart = convexPolygon2D.get(index);
         Point2DReadOnly edgeEnd = convexPolygon2D.get(next(index, numberOfVertices));

         isQueryOutsidePolygon |= isPoint2DOnSideOfLine2D(pointX, pointY, edgeStart, edgeEnd, clockwiseOrdered);
         minDistance = Math.min(minDistance, distanceSquaredFromPoint2DToLineSegment2D(pointX, pointY, edgeStart, edgeEnd));
      }

      minDistance = Math.sqrt(minDistance);

      if (!isQueryOutsidePolygon)
         minDistance = -minDistance;
      return minDistance;
   }

   /**
    * Returns minimum distance between the point and the polygon.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * The return value is negative if the point is inside the polygon.
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
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return the distance between the query and the polygon, it is negative if the point is inside
    *         the polygon.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static double signedDistanceFromPoint2DToConvexPolygon2D(Point2DReadOnly point, List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                                                   boolean clockwiseOrdered)
   {
      return signedDistanceFromPoint2DToConvexPolygon2D(point.getX(), point.getY(), convexPolygon2D, numberOfVertices, clockwiseOrdered);
   }

   /**
    * Computes the coordinates of the closest point to the ray that belongs to the given convex
    * polygon.
    * <p>
    * WARNINGS:
    * <ul>
    * <li>This method assumes that the given vertices already form a convex polygon.
    * <li>This methods assumes that the ray does not intersect with the polygon. Such scenario
    * should be handled with
    * {@link #intersectionBetweenRay2DAndConvexPolygon2D(Point2DReadOnly, Vector2DReadOnly, List, int, boolean, Point2DBasics, Point2DBasics)}.
    * </ul>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code false}.
    * <li>If the ray is parallel to the closest edge, the closest point to the ray origin is chosen.
    * </ul>
    * </p>
    *
    * @param rayOrigin the ray's origin. Not modified.
    * @param rayDirection the ray's direction. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @param closestPointToPack the point in which the coordinates of the closest point are stored.
    *           Modified.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static boolean closestPointToNonInterectingRay2D(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection,
                                                           List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices, boolean clockwiseOrdered,
                                                           Point2DBasics closestPointToPack)
   {
      int closestVertexIndexToLine = closestVertexIndexToLine2D(rayOrigin, rayDirection, convexPolygon2D, numberOfVertices);
      if (closestVertexIndexToLine == -1)
         return false;

      Point2DReadOnly closestVertexToLine = convexPolygon2D.get(closestVertexIndexToLine);
      boolean success = orthogonalProjectionOnConvexPolygon2D(rayOrigin, convexPolygon2D, numberOfVertices, clockwiseOrdered, closestPointToPack);
      if (!success)
         return false;

      Point2DReadOnly nextEdgeStart = closestVertexToLine;
      Point2DReadOnly nextEdgeEnd = convexPolygon2D.get(next(closestVertexIndexToLine, numberOfVertices));

      double nextEdgeDirectionX = nextEdgeEnd.getX() - nextEdgeStart.getX();
      double nextEdgeDirectionY = nextEdgeEnd.getY() - nextEdgeStart.getY();
      boolean areRayAndNextEdgeParallel = areVector2DsParallel(nextEdgeDirectionX, nextEdgeDirectionY, rayDirection.getX(), rayDirection.getY(),
                                                               ONE_TEN_MILLIONTH);

      if (areRayAndNextEdgeParallel)
         return true;

      Point2DReadOnly previousEdgeStart = convexPolygon2D.get(previous(closestVertexIndexToLine, numberOfVertices));
      Point2DReadOnly previousEdgeEnd = closestVertexToLine;

      double previousEdgeDirectionX = previousEdgeEnd.getX() - previousEdgeStart.getX();
      double previousEdgeDirectionY = previousEdgeEnd.getY() - previousEdgeStart.getY();
      boolean areRayAndPreviousEdgeParallel = areVector2DsParallel(previousEdgeDirectionX, previousEdgeDirectionY, rayDirection.getX(), rayDirection.getY(),
                                                                   ONE_TEN_MILLIONTH);

      if (areRayAndPreviousEdgeParallel)
         return true;

      double distanceToClosestVertex = distanceFromPoint2DToRay2D(closestVertexToLine, rayOrigin, rayDirection);
      double distanceToProjection = distanceFromPoint2DToRay2D(closestPointToPack, rayOrigin, rayDirection);

      if (distanceToClosestVertex < distanceToProjection)
         closestPointToPack.set(closestVertexToLine);

      return true;
   }

   /**
    * Computes the coordinates of the closest point to the ray that belongs to the given convex
    * polygon.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * WARNINGS:
    * <ul>
    * <li>This method assumes that the given vertices already form a convex polygon.
    * <li>This methods assumes that the ray does not intersect with the polygon. Such scenario
    * should be handled with
    * {@link #intersectionBetweenRay2DAndConvexPolygon2D(Point2DReadOnly, Vector2DReadOnly, List, int, boolean, Point2DBasics, Point2DBasics)}.
    * </ul>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code null}.
    * <li>If the ray is parallel to the closest edge, the closest point to the ray origin is chosen.
    * </ul>
    * </p>
    *
    * @param rayOrigin the ray's origin. Not modified.
    * @param rayDirection the ray's direction. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return the closest point to the ray or {@code null} if the method failed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static Point2D closestPointToNonInterectingRay2D(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection,
                                                           List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices, boolean clockwiseOrdered)
   {
      Point2D closestPoint = new Point2D();
      boolean success = closestPointToNonInterectingRay2D(rayOrigin, rayDirection, convexPolygon2D, numberOfVertices, clockwiseOrdered, closestPoint);
      return success ? closestPoint : null;
   }

   /**
    * Finds the index of the closest vertex to the given line.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code -1}.
    * </ul>
    * </p>
    *
    * @param pointOnLineX the x-coordinate of a point located on the line. Not modified.
    * @param pointOnLineY the y-coordinate of a point located on the line. Not modified.
    * @param lineDirectionX the x-coordinate of the direction of the line. Not modified.
    * @param lineDirectionY the y-coordinate of the direction of the line. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @return the index of the closest vertex to the query.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int closestVertexIndexToLine2D(double pointOnLineX, double pointOnLineY, double lineDirectionX, double lineDirectionY,
                                                List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);

      int index = -1;
      double minDistance = Double.POSITIVE_INFINITY;

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2DReadOnly vertex = convexPolygon2D.get(i);

         double distance = distanceFromPoint2DToLine2D(vertex.getX(), vertex.getY(), pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY);

         if (distance < minDistance)
         {
            index = i;
            minDistance = distance;
         }
      }

      return index;
   }

   /**
    * Finds the index of the closest vertex to the given line.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code -1}.
    * </ul>
    * </p>
    *
    * @param firstPointOnLine a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @return the index of the closest vertex to the query.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int closestVertexIndexToLine2D(Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine,
                                                List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices)
   {
      double pointOnLineX = firstPointOnLine.getX();
      double pointOnLineY = firstPointOnLine.getY();
      double lineDirectionX = secondPointOnLine.getX() - firstPointOnLine.getX();
      double lineDirectionY = secondPointOnLine.getY() - firstPointOnLine.getY();
      return closestVertexIndexToLine2D(pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, convexPolygon2D, numberOfVertices);
   }

   /**
    * Finds the index of the closest vertex to the given line.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code -1}.
    * </ul>
    * </p>
    *
    * @param pointOnLine a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @return the index of the closest vertex to the query.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int closestVertexIndexToLine2D(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection, List<? extends Point2DReadOnly> convexPolygon2D,
                                                int numberOfVertices)
   {
      return closestVertexIndexToLine2D(pointOnLine.getX(), pointOnLine.getY(), lineDirection.getX(), lineDirection.getY(), convexPolygon2D, numberOfVertices);
   }

   /**
    * Finds the index of the closest vertex to the given ray.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code -1}.
    * </ul>
    * </p>
    *
    * @param rayOrigin the ray's origin. Not modified.
    * @param rayDirection the ray's direction. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return the index of the closest vertex to the query.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int closestVertexIndexToRay2D(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection, List<? extends Point2DReadOnly> convexPolygon2D,
                                               int numberOfVertices, boolean clockwiseOrdered)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);

      int index = -1;
      double minDistance = Double.POSITIVE_INFINITY;

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2DReadOnly vertex = convexPolygon2D.get(i);

         double distance = distanceFromPoint2DToRay2D(vertex, rayOrigin, rayDirection);

         if (distance < minDistance)
         {
            index = i;
            minDistance = distance;
         }
      }

      return index;
   }

   /**
    * Finds the index of the closest vertex to the query.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code -1}.
    * </ul>
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @return the index of the closest vertex to the query.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int closestVertexIndexToPoint2D(Point2DReadOnly point, List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices)
   {
      return closestVertexIndexToPoint2D(point.getX(), point.getY(), convexPolygon2D, numberOfVertices);
   }

   /**
    * Finds the index of the closest vertex to the query.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code -1}.
    * </ul>
    * </p>
    *
    * @param pointX the x-coordinate of the query.
    * @param pointY the y-coordinate of the query.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @return the index of the closest vertex to the query.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int closestVertexIndexToPoint2D(double pointX, double pointY, List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);

      int index = -1;
      double minDistanceSquared = Double.POSITIVE_INFINITY;

      for (int i = 0; i < numberOfVertices; i++)
      {
         double distanceSquared = distanceSquaredBetweenPoint2Ds(pointX, pointY, convexPolygon2D.get(i));

         if (distanceSquared < minDistanceSquared)
         {
            index = i;
            minDistanceSquared = distanceSquared;
         }
      }

      return index;
   }

   /**
    * Finds the index of the closest edge to the query.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has one or no vertices, this method fails and returns {@code -1}.
    * </ul>
    * </p>
    *
    * @param pointX the x-coordinate of the query.
    * @param pointY the y-coordinate of the query.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return the index of the closest edge to the query.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int closestEdgeIndexToPoint2D(double pointX, double pointY, List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                               boolean clockwiseOrdered)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);

      if (numberOfVertices <= 1)
         return -1;

      boolean isQueryOutsidePolygon = false;
      int insideIndex = -1;
      int outsideIndex = -1;
      double minOutsideDistanceSquared = Double.POSITIVE_INFINITY;
      double minInsideDistanceSquared = Double.POSITIVE_INFINITY;

      for (int edgeIndex = 0; edgeIndex < numberOfVertices; edgeIndex++)
      {
         Point2DReadOnly edgeStart = convexPolygon2D.get(edgeIndex);
         Point2DReadOnly edgeEnd = convexPolygon2D.get(next(edgeIndex, numberOfVertices));

         double distanceSquared = distanceSquaredFromPoint2DToLineSegment2D(pointX, pointY, edgeStart, edgeEnd);

         boolean isOutsideEdge = isPoint2DOnSideOfLine2D(pointX, pointY, edgeStart, edgeEnd, clockwiseOrdered);
         isQueryOutsidePolygon |= isOutsideEdge;

         /*
          * By keeping track of whether the point is outside or inside w.r.t. to each edge, it is
          * ensured that if the point is outside the polygon the edge returned is visible from the
          * query.
          */
         if (isOutsideEdge)
         {
            if (distanceSquared < minOutsideDistanceSquared)
            {
               outsideIndex = edgeIndex;
               minOutsideDistanceSquared = distanceSquared;
            }
         }
         else
         {
            if (distanceSquared < minInsideDistanceSquared)
            {
               insideIndex = edgeIndex;
               minInsideDistanceSquared = distanceSquared;
            }
         }
      }

      return isQueryOutsidePolygon ? outsideIndex : insideIndex;
   }

   /**
    * Finds the index of the closest edge to the query.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has one or no vertices, this method fails and returns {@code -1}.
    * </ul>
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return the index of the closest edge to the query.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int closestEdgeIndexToPoint2D(Point2DReadOnly point, List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                               boolean clockwiseOrdered)
   {
      return closestEdgeIndexToPoint2D(point.getX(), point.getY(), convexPolygon2D, numberOfVertices, clockwiseOrdered);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the index of the
    * first vertex that is in the line-of-sight.
    * <p>
    * WARNING:
    * <ul>
    * <li>This method assumes that the given vertices already form a convex polygon.
    * <li>This method assumes that the given observer is located outside the polygon.
    * </ul>
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
    * @param observerX the x-coordinate of the observer.
    * @param observerY the y-coordinate of the observer.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return the index of the first vertex that is in the line-of-sight, {@code -1} if this method
    *         fails.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int lineOfSightStartIndex(double observerX, double observerY, List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                           boolean clockwiseOrdered)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);

      if (numberOfVertices == 0)
         return -1;
      if (numberOfVertices == 1)
      {
         Point2DReadOnly vertex = convexPolygon2D.get(0);
         if (vertex.getX() == observerX && vertex.getY() == observerY)
            return -1;
         else
            return 0;
      }
      if (numberOfVertices == 2)
      {
         if (isPoint2DInsideConvexPolygon2D(observerX, observerY, convexPolygon2D, numberOfVertices, clockwiseOrdered, 0.0))
            return -1;
         boolean isOnSide = isPoint2DOnSideOfLine2D(observerX, observerY, convexPolygon2D.get(0), convexPolygon2D.get(1), clockwiseOrdered);
         return isOnSide ? 0 : 1;
      }

      boolean previousEdgeVisible = canObserverSeeEdge(numberOfVertices - 1, observerX, observerY, convexPolygon2D, numberOfVertices, clockwiseOrdered);

      for (int edgeIndex = 0; edgeIndex < numberOfVertices; edgeIndex++)
      {
         boolean edgeVisible = canObserverSeeEdge(edgeIndex, observerX, observerY, convexPolygon2D, numberOfVertices, clockwiseOrdered);
         if (!previousEdgeVisible && edgeVisible)
            return edgeIndex;
         previousEdgeVisible = edgeVisible;
      }
      return -1;
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the index of the
    * first vertex that is in the line-of-sight.
    * <p>
    * WARNING:
    * <ul>
    * <li>This method assumes that the given vertices already form a convex polygon.
    * <li>This method assumes that the given observer is located outside the polygon.
    * </ul>
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
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return the index of the first vertex that is in the line-of-sight, {@code -1} if this method
    *         fails.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int lineOfSightStartIndex(Point2DReadOnly observer, List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                           boolean clockwiseOrdered)
   {
      return lineOfSightStartIndex(observer.getX(), observer.getY(), convexPolygon2D, numberOfVertices, clockwiseOrdered);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the index of the
    * last vertex that is in the line-of-sight.
    * <p>
    * WARNING:
    * <ul>
    * <li>This method assumes that the given vertices already form a convex polygon.
    * <li>This method assumes that the given observer is located outside the polygon.
    * </ul>
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
    * @param observerX the x-coordinate of the observer.
    * @param observerY the y-coordinate of the observer.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return the index of the last vertex that is in the line-of-sight, {@code -1} if this method
    *         fails.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int lineOfSightEndIndex(double observerX, double observerY, List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                         boolean clockwiseOrdered)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);

      if (numberOfVertices == 0)
         return -1;
      if (numberOfVertices == 1)
      {
         Point2DReadOnly vertex = convexPolygon2D.get(0);
         if (vertex.getX() == observerX && vertex.getY() == observerY)
            return -1;
         else
            return 0;
      }
      if (numberOfVertices == 2)
      {
         if (isPoint2DInsideConvexPolygon2D(observerX, observerY, convexPolygon2D, numberOfVertices, clockwiseOrdered, 0.0))
            return -1;
         boolean isOnSide = isPoint2DOnSideOfLine2D(observerX, observerY, convexPolygon2D.get(0), convexPolygon2D.get(1), clockwiseOrdered);
         return isOnSide ? 1 : 0;
      }

      boolean previousEdgeVisible = canObserverSeeEdge(numberOfVertices - 1, observerX, observerY, convexPolygon2D, numberOfVertices, clockwiseOrdered);

      for (int edgeIndex = 0; edgeIndex < numberOfVertices; edgeIndex++)
      {
         boolean edgeVisible = canObserverSeeEdge(edgeIndex, observerX, observerY, convexPolygon2D, numberOfVertices, clockwiseOrdered);
         if (previousEdgeVisible && !edgeVisible)
            return edgeIndex;
         previousEdgeVisible = edgeVisible;
      }
      return -1;
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the index of the
    * last vertex that is in the line-of-sight.
    * <p>
    * WARNING:
    * <ul>
    * <li>This method assumes that the given vertices already form a convex polygon.
    * <li>This method assumes that the given observer is located outside the polygon.
    * </ul>
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
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return the index of the last vertex that is in the line-of-sight, {@code -1} if this method
    *         fails.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int lineOfSightEndIndex(Point2DReadOnly observer, List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                         boolean clockwiseOrdered)
   {
      return lineOfSightEndIndex(observer.getX(), observer.getY(), convexPolygon2D, numberOfVertices, clockwiseOrdered);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the indices of the
    * first vertex and last vertex that are in the line-of-sight.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * WARNING:
    * <ul>
    * <li>This method assumes that the given vertices already form a convex polygon.
    * <li>This method assumes that the given observer is located outside the polygon.
    * </ul>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code null}.
    * <li>The observer is inside the polygon, this method fails and returns {@code null}.
    * <li>The polygon has exactly one vertex, this method returns {@code new int[]{0, 0}} if the
    * observer is different from the polygon's vertex, or returns {@code null} if the observer is
    * equal to the polygon's vertex.
    * </ul>
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return in order the index of the first vertex and the index of the last vertex that are in
    *         the line-of-sight, {@code null} if this method fails.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int[] lineOfSightIndices(Point2DReadOnly observer, List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                          boolean clockwiseOrdered)
   {
      int lineOfSightStartIndex = lineOfSightStartIndex(observer, convexPolygon2D, numberOfVertices, clockwiseOrdered);
      int lineOfSightEndIndex = lineOfSightEndIndex(observer, convexPolygon2D, numberOfVertices, clockwiseOrdered);
      if (lineOfSightStartIndex == -1 || lineOfSightEndIndex == -1)
         return null;
      else
         return new int[] {lineOfSightStartIndex, lineOfSightEndIndex};
   }

   /**
    * Finds the index of the next polygon's edge that the given line intersects.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * The exploration starts with the edge at the index {@code previousEdgeIndex + 1}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If {@code previousEdgeIndex == -2}, this method automatically fails and returns
    * {@code -2}.
    * <li>If the polygon has less than 2 vertices, this method fails and returns {@code -2}.
    * <li>If the line and an edge of the polygon are collinear and the polygon has exactly 2
    * vertices, this method fails and returns {@code -2}.
    * <li>If the polygon has exactly 2 vertices and the line intersects it, this method returns
    * {@code previousEdgeIndex + 1}.
    * <li>If the line and an edge of the polygon are collinear and the polygon has more than 2
    * vertices, this method returns the index of the previous edge or the next edge if the given
    * {@code previousEdgeIndex} refers to the previous edge.
    * <li>If the line is going through a vertex, this method considers that the line intersects with
    * the two adjacent edges.
    * </ul>
    * </p>
    *
    * @param previousEdgeIndex refers to the index of the previously found edge intersecting with
    *           the line. To find the first intersecting edge, it should be set to {@code -1}.
    * @param pointOnLine a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @return the index of the next polygon's edge the line intersects with, if none found
    *         {@code -2}.
    * @throws IndexOutOfBoundsException if {@code previousEdgeIndex} is not in [-2,
    *            {@code numberOfVertices}[.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int nextEdgeIndexIntersectingWithLine2D(int previousEdgeIndex, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection,
                                                         List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices)
   {
      return nextEdgeIndexIntersectingWithLine2D(previousEdgeIndex, pointOnLine.getX(), pointOnLine.getY(), lineDirection.getX(), lineDirection.getY(),
                                                 convexPolygon2D, numberOfVertices);
   }

   /**
    * Finds the index of the next polygon's edge that the given line intersects.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * The exploration starts with the edge at the index {@code previousEdgeIndex + 1}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If {@code previousEdgeIndex == -2}, this method automatically fails and returns
    * {@code -2}.
    * <li>If the polygon has less than 2 vertices, this method fails and returns {@code -2}.
    * <li>If the line and an edge of the polygon are collinear and the polygon has exactly 2
    * vertices, this method fails and returns {@code -2}.
    * <li>If the polygon has exactly 2 vertices and the line intersects it, this method returns
    * {@code previousEdgeIndex + 1}.
    * <li>If the line and an edge of the polygon are collinear and the polygon has more than 2
    * vertices, this method returns the index of the previous edge or the next edge if the given
    * {@code previousEdgeIndex} refers to the previous edge.
    * <li>If the line is going through a vertex, this method considers that the line intersects with
    * the two adjacent edges.
    * </ul>
    * </p>
    *
    * @param previousEdgeIndex refers to the index of the previously found edge intersecting with
    *           the line. To find the first intersecting edge, it should be set to {@code -1}.
    * @param pointOnLineX the x-coordinate of a point located on the line.
    * @param pointOnLineY the y-coordinate of a point located on the line.
    * @param lineDirectionX the x-component of the direction of the line.
    * @param lineDirectionY the y-component of the direction of the line.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @return the index of the next polygon's edge the line intersects with, if none found
    *         {@code -2}.
    * @throws IndexOutOfBoundsException if {@code previousEdgeIndex} is not in [-2,
    *            {@code numberOfVertices}[.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static int nextEdgeIndexIntersectingWithLine2D(int previousEdgeIndex, double pointOnLineX, double pointOnLineY, double lineDirectionX,
                                                         double lineDirectionY, List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);
      if (previousEdgeIndex < -2 || previousEdgeIndex >= numberOfVertices)
         throw new IndexOutOfBoundsException("Expected previousEdgeIndex to be in [-2; numberOfVertices[, but was: " + previousEdgeIndex);

      if (numberOfVertices <= 1 || previousEdgeIndex == -2)
         return -2;

      if (numberOfVertices == 2)
      {
         if (previousEdgeIndex == 0) // Meaning the line intersects the polygon and that the next index is 1.
            return 1;

         Point2DReadOnly edgeStart = convexPolygon2D.get(0);
         Point2DReadOnly edgeEnd = convexPolygon2D.get(1);
         if (doLine2DAndLineSegment2DIntersect(pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, edgeStart, edgeEnd))
         { // Getting here means that this is the first time the method is being called.
            return 0;
         }
      }

      int edgeIndex = previousEdgeIndex;

      for (int i = 0; i < numberOfVertices; i++)
      {
         edgeIndex = next(edgeIndex, numberOfVertices);
         Point2DReadOnly edgeStart = convexPolygon2D.get(edgeIndex);
         Point2DReadOnly edgeEnd = convexPolygon2D.get(next(edgeIndex, numberOfVertices));

         boolean doLineAndEdgeIntersect = doLine2DAndLineSegment2DIntersect(pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, edgeStart, edgeEnd);
         if (doLineAndEdgeIntersect)
            return edgeIndex;
      }

      return -2;
   }

   /**
    * Computes the orthogonal projection of a 2D point on a given 2D convex polygon.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code false}.
    * <li>If the polygon has exactly one vertex, the result is the polygon only vertex, this method
    * returns {@code true}.
    * </ul>
    * </p>
    *
    * @param pointToProjectX the x-coordinate of the point to compute the projection of.
    * @param pointToProjectY the y-coordinate of the point to compute the projection of.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @param projectionToPack point in which the projection of the point onto the convex polygon is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static boolean orthogonalProjectionOnConvexPolygon2D(double pointToProjectX, double pointToProjectY, List<? extends Point2DReadOnly> convexPolygon2D,
                                                               int numberOfVertices, boolean clockwiseOrdered, Point2DBasics projectionToPack)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);

      if (numberOfVertices == 0)
         return false;

      if (numberOfVertices == 1)
      {
         projectionToPack.set(convexPolygon2D.get(0));
         return true;
      }

      if (numberOfVertices == 2)
      {
         return orthogonalProjectionOnLineSegment2D(pointToProjectX, pointToProjectY, convexPolygon2D.get(0), convexPolygon2D.get(1), projectionToPack);
      }

      int closestEdgeIndex = closestEdgeIndexToPoint2D(pointToProjectX, pointToProjectY, convexPolygon2D, numberOfVertices, clockwiseOrdered);

      if (closestEdgeIndex == -1)
         return false;

      if (!canObserverSeeEdge(closestEdgeIndex, pointToProjectX, pointToProjectY, convexPolygon2D, numberOfVertices, clockwiseOrdered))
         return false;

      Point2DReadOnly edgeStart = convexPolygon2D.get(closestEdgeIndex);
      Point2DReadOnly edgeEnd = convexPolygon2D.get(next(closestEdgeIndex, numberOfVertices));

      return orthogonalProjectionOnLineSegment2D(pointToProjectX, pointToProjectY, edgeStart, edgeEnd, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on a given 2D convex polygon.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
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
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @param projectionToPack point in which the projection of the point onto the convex polygon is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static boolean orthogonalProjectionOnConvexPolygon2D(Point2DReadOnly pointToProject, List<? extends Point2DReadOnly> convexPolygon2D,
                                                               int numberOfVertices, boolean clockwiseOrdered, Point2DBasics projectionToPack)
   {
      return orthogonalProjectionOnConvexPolygon2D(pointToProject.getX(), pointToProject.getY(), convexPolygon2D, numberOfVertices, clockwiseOrdered,
                                                   projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on a given 2D convex polygon.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
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
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return the coordinates of the projection, or {@code null} if the method failed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static Point2D orthogonalProjectionOnConvexPolygon2D(Point2DReadOnly pointToProject, List<? extends Point2DReadOnly> convexPolygon2D,
                                                               int numberOfVertices, boolean clockwiseOrdered)
   {
      Point2D projection = new Point2D();
      boolean success = orthogonalProjectionOnConvexPolygon2D(pointToProject.getX(), pointToProject.getY(), convexPolygon2D, numberOfVertices, clockwiseOrdered,
                                                              projection);
      return success ? projection : null;
   }

   /**
    * Determines whether an observer can see the outside of the given edge of the given convex
    * polygon.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * The edge is defined by its start {@code convexPolygon2D.get(edgeIndex)} and its end
    * {@code convexPolygon2D.get(edgeIndex + 1)}.
    * </p>
    *
    * @param edgeIndex the vertex index of the start of the edge.
    * @param observer the coordinates of the observer. Not modified.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return {@code true} if the observer can see the outside of the edge, {@code false} if the
    *         observer cannot see the outside or is lying on the edge.
    * @throws IndexOutOfBoundsException if {@code edgeIndex} is either negative or greater or equal
    *            than {@code numberOfVertices}.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static boolean canObserverSeeEdge(int edgeIndex, Point2DReadOnly observer, List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices,
                                            boolean clockwiseOrdered)
   {
      return canObserverSeeEdge(edgeIndex, observer.getX(), observer.getY(), convexPolygon2D, numberOfVertices, clockwiseOrdered);
   }

   /**
    * Determines whether an observer can see the outside of the given edge of the given convex
    * polygon.
    * <p>
    * WARNING: This method assumes that the given vertices already form a convex polygon.
    * </p>
    * <p>
    * The edge is defined by its start {@code convexPolygon2D.get(edgeIndex)} and its end
    * {@code convexPolygon2D.get(edgeIndex + 1)}.
    * </p>
    *
    * @param edgeIndex the vertex index of the start of the edge.
    * @param observerX the x-coordinate of the observer.
    * @param observerY the y-coordinate of the observer.
    * @param convexPolygon2D the list containing in [0, {@code numberOfVertices}[ the vertices of
    *           the convex polygon. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return {@code true} if the observer can see the outside of the edge, {@code false} if the
    *         observer cannot see the outside or is lying on the edge.
    * @throws IndexOutOfBoundsException if {@code edgeIndex} is either negative or greater or equal
    *            than {@code numberOfVertices}.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public static boolean canObserverSeeEdge(int edgeIndex, double observerX, double observerY, List<? extends Point2DReadOnly> convexPolygon2D,
                                            int numberOfVertices, boolean clockwiseOrdered)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);
      checkEdgeIndex(edgeIndex, numberOfVertices);
      Point2DReadOnly edgeStart = convexPolygon2D.get(edgeIndex);
      Point2DReadOnly edgeEnd = convexPolygon2D.get(next(edgeIndex, numberOfVertices));
      return isPoint2DOnSideOfLine2D(observerX, observerY, edgeStart, edgeEnd, clockwiseOrdered);
   }

   /**
    * Sorts the vertices to complete the first step of the Graham scan algorithm.
    * <p>
    * First the vertex located at the lowest x-coordinate is found and used as the reference P.
    * </p>
    * <p>
    * Then the vertices are sorted in increasing order of the angle they and the reference P make
    * with the y-axis.
    * </p>
    * <p>
    * The <a href="https://en.wikipedia.org/wiki/Quicksort#Choice_of_pivot">Quicksort algorithm</a>
    * is adapted to the Graham scan application to prevent garbage generation.
    * </p>
    *
    * @param vertices the list of vertices to be sorted. Modified.
    * @param numberOfVertices specifies the number of relevant points in the list. The algorithm
    *           will only process the points &in; [0; {@code numberOfVertices}[.
    */
   static void grahamScanAngleSort(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      Point2DReadOnly minXMaxYVertex = vertices.get(findMinXMaxYVertexIndex(vertices, numberOfVertices));
      grahamScanAngleSortRecursive(vertices, minXMaxYVertex, 0, numberOfVertices - 1);
   }

   /**
    * Recursive step of the sorting algorithm for the Graham scan.
    */
   private static void grahamScanAngleSortRecursive(List<? extends Point2DReadOnly> vertices, Point2DReadOnly minXMaxYVertex, int startIndex, int endIndex)
   {
      if (endIndex <= startIndex)
         return;

      // Choose the pivot randomly
      int pivotIndex = random.nextInt(endIndex - startIndex) + startIndex;

      /*
       * Partitioning:
       * @formatter:off
       * +------------------------+
       *  | <= pivot | > pivot |
       * +------------------------+
       *             ^
       *             |
       *         pivotIndex
       * @formatter:on
       */
      pivotIndex = grahamScanAnglePartition(vertices, minXMaxYVertex, startIndex, endIndex, pivotIndex);
      // Recurse on each side of the pivot
      grahamScanAngleSortRecursive(vertices, minXMaxYVertex, startIndex, pivotIndex - 1);
      grahamScanAngleSortRecursive(vertices, minXMaxYVertex, pivotIndex + 1, endIndex);
   }

   /**
    * Partition the list around the pivot:
    *
    * <pre>
    * +------------------------+
    * |  <= pivot  |  > pivot  |
    * +------------------------+
    *              ^
    *              |
    *          pivotIndex
    * </pre>
    */
   private static int grahamScanAnglePartition(List<? extends Point2DReadOnly> list, Point2DReadOnly minXMaxYVertex, int startIndex, int endIndex,
                                               int pivotIndex)
   {
      Point2DReadOnly pivot = list.get(pivotIndex);
      // Push the pivot to the endIndex
      Collections.swap(list, pivotIndex, endIndex);
      // Index where the array is partitioned between smaller and larger elements than the pivot
      int partitionIndex = startIndex;

      for (int i = startIndex; i < endIndex; i++)
      {
         if (grahamScanAngleCompare(minXMaxYVertex, list.get(i), pivot) <= 0)
         { // Element is lower or equal to the pivot: it is part of the first partition.
            Collections.swap(list, i, partitionIndex);
            partitionIndex++;
         }
      }

      // Move the pivot back to the partition index.
      Collections.swap(list, endIndex, partitionIndex);
      return partitionIndex;
   }

   static int grahamScanAngleCompare(Point2DReadOnly minXMaxYVertex, Point2DReadOnly vertex1, Point2DReadOnly vertex2)
   {
      if (vertex1 == minXMaxYVertex)
         return -1;
      if (vertex2 == minXMaxYVertex)
         return 1;
      if (vertex1.epsilonEquals(vertex2, EPSILON))
         return 0;
      if (isPoint2DOnLeftSideOfLine2D(vertex1, minXMaxYVertex, vertex2))
         return -1;
      return 1;
   }

   /**
    * Moves the element located at {@code indexOfElementToShift} to {@code listSize - 1} and shifts
    * all the elements located in [{@code indexToRemove + 1}; {@code listSize - 1}] by {@code -1}.
    *
    * @param list the list from which the element is to be moved. Modified.
    * @param indexOfElementToMove the index of the element to move to the end of the list.
    * @param listSize the actual size of the list with relevant information.
    */
   static <T> void moveElementToEnd(List<T> list, int indexOfElementToMove, int listSize)
   {
      T elementToMove = list.get(indexOfElementToMove);

      while (indexOfElementToMove < listSize - 1)
         list.set(indexOfElementToMove, list.get(++indexOfElementToMove));

      list.set(indexOfElementToMove, elementToMove);
   }

   /**
    * Finds the index of the vertex with the lowest x-coordinate. If the lowest x-coordinate exists
    * in more than one vertex in the list, the vertex with the highest y-coordinate out of the
    * candidates is chosen.
    *
    * @param vertices the list of vertices to search in. Not modified.
    * @param numberOfVertices the number of relevant vertices to search. This method searches in the
    *           range [{@code 0}, {@code numberOfVertices}].
    * @return the index of the vertex with min x-coordinate.
    */
   static int findMinXMaxYVertexIndex(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return -1;

      int minXMaxYIndex = 0;
      Point2DReadOnly minXMaxY = vertices.get(minXMaxYIndex);

      for (int vertexIndex = 1; vertexIndex < numberOfVertices; vertexIndex++)
      {
         Point2DReadOnly candidate = vertices.get(vertexIndex);

         if (candidate.getX() < minXMaxY.getX())
         {
            minXMaxYIndex = vertexIndex;
            minXMaxY = candidate;
         }
         else if (candidate.getX() == minXMaxY.getX() && candidate.getY() > minXMaxY.getY())
         {
            minXMaxYIndex = vertexIndex;
            minXMaxY = candidate;
         }
      }

      return minXMaxYIndex;
   }

   /**
    * Finds the index of a vertex in the specified supplier given search criteria.
    * 
    * @param vertex2DSupplier the vertex supplier containing vertices to be search through.
    * @param isXPriority whether the search should focus first on finding the vertex with the "best"
    *           x-coordinate, or y-coordinates.
    * @param xBound the search criterion for the x-coordinate, for instance {@link Bound#MAX} while
    *           result in searching the vertex with the lowest x value.
    * @param yBound the search criterion for the Y-coordinate, for instance {@link Bound#MIN} while
    *           result in searching the vertex with the highest y value.
    * 
    * @return the index in the supplier of the best vertex according to the given criteria.
    */
   public static int findVertexIndex(Vertex2DSupplier vertex2DSupplier, boolean isXPriority, Bound xBound, Bound yBound)
   {
      if (vertex2DSupplier.getNumberOfVertices() == 0)
         return -1;

      Bound bound1 = isXPriority ? xBound : yBound; // The bound to use for the high-priority coordinate.
      Bound bound2 = isXPriority ? yBound : xBound; // The bound to use for the low-priority coordinate.

      int coord1 = isXPriority ? 0 : 1; // The high-priority coordinate index (x or y).
      int coord2 = isXPriority ? 1 : 0; // The low-priority coordinate index (x or y).

      int bestIndex = 0;
      Point2DReadOnly vertex = vertex2DSupplier.getVertex(bestIndex);
      double bestCoord1 = vertex.getElement(coord1);
      double bestCoord2 = vertex.getElement(coord2);

      for (int vertexIndex = 1; vertexIndex < vertex2DSupplier.getNumberOfVertices(); vertexIndex++)
      {
         vertex = vertex2DSupplier.getVertex(vertexIndex);

         double candidateCoord1 = vertex.getElement(coord1);
         double candidateCoord2 = vertex.getElement(coord2);

         if (bound1.isFirstBetter(candidateCoord1, bestCoord1))
         {
            bestIndex = vertexIndex;
            bestCoord1 = candidateCoord1;
            bestCoord2 = candidateCoord2;
         }
         else if (candidateCoord1 == bestCoord1 && bound2.isFirstBetter(candidateCoord2, bestCoord2))
         {
            bestIndex = vertexIndex;
            bestCoord1 = candidateCoord1;
            bestCoord2 = candidateCoord2;
         }
      }

      return bestIndex;
   }

   /**
    * Recomputes the given {@code index} such that it is &in; [0, {@code listSize}[.
    * <p>
    * The {@code index} remains unchanged if already &in; [0, {@code listSize}[.
    * <p>
    * Examples:
    * <ul>
    * <li>{@code wrap(-1, 10)} returns 9.
    * <li>{@code wrap(10, 10)} returns 0.
    * <li>{@code wrap( 5, 10)} returns 5.
    * <li>{@code wrap(15, 10)} returns 5.
    * </ul>
    * </p>
    *
    * @param index the index to be wrapped if necessary.
    * @param listSize the size of the list around which the index is to be wrapped.
    * @return the wrapped index.
    */
   public static int wrap(int index, int listSize)
   {
      index %= listSize;
      if (index < 0)
         index += listSize;
      return index;
   }

   /**
    * Increments then recomputes the given {@code index} such that it is &in; [0, {@code listSize}[.
    * Examples:
    * <ul>
    * <li>{@code next(-1, 10)} returns 0.
    * <li>{@code next(10, 10)} returns 1.
    * <li>{@code next( 5, 10)} returns 6.
    * <li>{@code next(15, 10)} returns 6.
    * </ul>
    * </p>
    *
    * @param index the index to be incremented and wrapped if necessary.
    * @param listSize the size of the list around which the index is to be wrapped.
    * @return the wrapped incremented index.
    */
   public static int next(int index, int listSize)
   {
      return wrap(index + 1, listSize);
   }

   /**
    * Decrements then recomputes the given {@code index} such that it is &in; [0, {@code listSize}[.
    * Examples:
    * <ul>
    * <li>{@code next(-1, 10)} returns 10.
    * <li>{@code next(10, 10)} returns 9.
    * <li>{@code next( 5, 10)} returns 4.
    * <li>{@code next(15, 10)} returns 4.
    * </ul>
    * </p>
    *
    * @param index the index to be decremented and wrapped if necessary.
    * @param listSize the size of the list around which the index is to be wrapped.
    * @return the wrapped decremented index.
    */
   public static int previous(int index, int listSize)
   {
      return wrap(index - 1, listSize);
   }

   private static void checkEdgeIndex(int edgeIndex, int numberOfVertices)
   {
      if (edgeIndex < 0 || edgeIndex >= numberOfVertices)
         throw new IndexOutOfBoundsException("Expected edgeIndex to be in [0; numberOfVertices[, but was: " + edgeIndex);
   }

   private static void checkNumberOfVertices(List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > convexPolygon2D.size())
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + convexPolygon2D.size() + "].");
   }
}
