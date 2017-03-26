package us.ihmc.euclid.geometry.tools;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D;

import java.util.Collections;
import java.util.List;
import java.util.Random;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class EuclidGeometryPolygonTools
{
   private static final Random random = new Random();
   static final double EPSILON = 1.0e-7;

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
    * @throws ArrayIndexOutOfBoundsException if {@code numberOfVertices} is negative or greater than
    *            the size of the given list of vertices.
    */
   public static int inPlaceGiftWrapConvexHull2D(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      if (numberOfVertices <= 0 || numberOfVertices > vertices.size())
         throw new ArrayIndexOutOfBoundsException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + vertices.size() + "].");

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
         }

         for (int vertexIndex = lastHullVertexIndex + 2; vertexIndex <= numberOfVertices; vertexIndex++)
         {
            int wrappedIndex = wrap(vertexIndex, numberOfVertices);
            Point2DReadOnly vertex = vertices.get(wrappedIndex);

            if (vertex.epsilonEquals(candidateVertex, EPSILON) || (wrappedIndex != 0 && vertex.epsilonEquals(firstVertex, EPSILON)))
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
    * @param numberOfVertices specifies the number of relevant points in the list. The algorithm
    *           will only process the points &in; [0; {@code numberOfVertices}[.
    * @return the size of the convex hull.
    * @throws ArrayIndexOutOfBoundsException if {@code numberOfVertices} is negative or greater than
    *            the size of the given list of vertices.
    */
   public static int inPlaceGrahamScanConvexHull2D(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      if (numberOfVertices <= 0 || numberOfVertices > vertices.size())
         throw new ArrayIndexOutOfBoundsException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + vertices.size() + "].");

      grahamScanAngleSort(vertices, numberOfVertices);

      if (numberOfVertices <= 3)
         return numberOfVertices;

      int currentIndex = 1;

      while (currentIndex < numberOfVertices)
      {
         if (isConvexAtVertex(currentIndex, vertices, numberOfVertices, true))
         { // Convex at the current vertex: move on to next. 
            currentIndex++;
         }
         else
         { // Not convex: remove the current vertex.
            remove(vertices, currentIndex, numberOfVertices);
            numberOfVertices--; // The vertex is not part of the convex hull.
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
       * // @formatter:off
       * +------------------------+
       * |  <= pivot  |  > pivot  |
       * +------------------------+
       *              ^
       *              |
       *          pivotIndex
       * // @formatter:on
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
      if (EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(vertex1, minXMaxYVertex, vertex2))
         return -1;
      return 1;
   }

   /**
    * Moves the element located at {@code indexToRemove} to {@code listSize - 1} and shifts all the
    * elements located in [{@code indexToRemove + 1}; {@code listSize - 1}] by {@code -1}.
    * 
    * @param list the list from which the element is to be removed. Modified.
    * @param indexToRemove the index of the element to remove.
    * @param listSize the actual size of the list with relevant information.
    */
   static <T> void remove(List<T> list, int indexToRemove, int listSize)
   {
      T elementToRemove = list.get(indexToRemove);

      while (indexToRemove < listSize - 1)
         list.set(indexToRemove, list.get(++indexToRemove));

      list.set(indexToRemove, elementToRemove);
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
    * Tests if the polygon defined by the given {@code vertices} is convex at the vertex defined by
    * the given {@code vertexIndex}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>the method returns {@code false} if the vertex, next and previous vertices lie on the same
    * line.
    * </ul>
    * </p>
    * 
    * @param vertexIndex the index of the vertex to be tested for convexity.
    * @param vertices the list of vertices defining the polygon to test. Not modified.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return {@code true} if the polygon is convex at the given vertex, {@code false} otherwise.
    */
   public static boolean isConvexAtVertex(int vertexIndex, List<? extends Point2DReadOnly> vertices, boolean clockwiseOrdered)
   {
      return isConvexAtVertex(vertexIndex, vertices, vertices.size(), clockwiseOrdered);
   }

   /**
    * Tests if the polygon defined by the given {@code vertices} is convex at the vertex defined by
    * the given {@code vertexIndex}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>the method returns {@code false} if the vertex, next and previous vertices lie on the same
    * line.
    * </ul>
    * </p>
    * 
    * @param vertexIndex the index of the vertex to be tested for convexity.
    * @param vertices the list of vertices defining the polygon to test. Not modified.
    * @param numberOfVertices the number of vertices relevant to the polygon.
    * @param clockwiseOrdered whether the vertices are clockwise or counter-clockwise ordered.
    * @return {@code true} if the polygon is convex at the given vertex, {@code false} otherwise.
    */
   public static boolean isConvexAtVertex(int vertexIndex, List<? extends Point2DReadOnly> vertices, int numberOfVertices, boolean clockwiseOrdered)
   {
      Point2DReadOnly vertex = vertices.get(vertexIndex);
      Point2DReadOnly previousVertex = vertices.get(previous(vertexIndex, numberOfVertices));
      Point2DReadOnly nextVertex = vertices.get(next(vertexIndex, numberOfVertices));

      return EuclidGeometryTools.isPoint2DOnSideOfLine2D(vertex, previousVertex, nextVertex, clockwiseOrdered);
   }

   /**
    * Recomputes the given {@code index} such that it is &in; [0, {@code listSize}[.
    * <p>
    * The {@code index} remains unchanged if already &in; [0, {@code listSize}[.
    * <p>
    * Examples:
    * <ul>
    * <li> {@code wrap(-1, 10)} returns 9.
    * <li> {@code wrap(10, 10)} returns 0.
    * <li> {@code wrap( 5, 10)} returns 5.
    * <li> {@code wrap(15, 10)} returns 5.
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
    * <li> {@code next(-1, 10)} returns 0.
    * <li> {@code next(10, 10)} returns 1.
    * <li> {@code next( 5, 10)} returns 6.
    * <li> {@code next(15, 10)} returns 6.
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
    * <li> {@code next(-1, 10)} returns 10.
    * <li> {@code next(10, 10)} returns 9.
    * <li> {@code next( 5, 10)} returns 4.
    * <li> {@code next(15, 10)} returns 4.
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
}
