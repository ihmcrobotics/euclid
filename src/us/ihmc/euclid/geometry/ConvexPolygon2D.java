package us.ihmc.euclid.geometry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import us.ihmc.euclid.geometry.exceptions.EmptyPolygonException;
import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * Describes a planar convex polygon defined in the XY-plane.
 * <p>
 * The vertices of a convex polygon are clockwise and are all different.
 * </p>
 * <p>
 * This implementation of convex polygon is designed for garbage free operations.
 * </p>
 */
public class ConvexPolygon2D implements ConvexPolygon2DBasics, GeometryObject<ConvexPolygon2D>
{
   /**
    * Field for future expansion of {@code ConvexPolygon2d} to enable having the vertices in clockwise
    * or counter-clockwise ordered.
    */
   private final boolean clockwiseOrdered = true;
   /**
    * The current size or number of vertices for this convex polygon.
    */
   private int numberOfVertices = 0;
   /**
    * The internal memory of {@code ConvexPolygon2d}.
    * <p>
    * New vertices can be added to this polygon, after which the method {@link #update()} has to be
    * called to ensure that this polygon is convex.
    * </p>
    * <p>
    * Note that this list is used as a buffer to recycle the memory and can thus be greater than the
    * actual size of this polygon.
    * </p>
    * <p>
    * The vertices composing this polygon are located in the index range [0, {@link #numberOfVertices}[
    * in this list.
    * </p>
    */
   private final List<Point2D> clockwiseOrderedVertices = new ArrayList<>();
   private final List<Point2D> unmodifiableVertexBuffer = Collections.unmodifiableList(clockwiseOrderedVertices);
   /**
    * The smallest axis-aligned bounding box that contains all this polygon's vertices.
    * <p>
    * It is updated in the method {@link #updateBoundingBox()} which is itself called in
    * {@link #update()}.
    * </p>
    */
   private final BoundingBox2D boundingBox = new BoundingBox2D();
   /**
    * The centroid of this polygon which is located at the center of mass of this polygon when
    * considered as a physical object with constant thickness and density.
    * <p>
    * It is updated in the method {@link #updateCentroidAndArea()} which is itself called in
    * {@link #update()}.
    * </p>
    */
   private final Point2D centroid = new Point2D();
   /**
    * The area of this convex polygon.
    * <p>
    * It is updated in the method {@link #updateCentroidAndArea()} which is itself called in
    * {@link #update()}.
    * </p>
    * <p>
    * When a polygon is empty, i.e. has no vertices, the area is equal to {@link Double#NaN}.
    * </p>
    */
   private double area;
   /**
    * This field is used to know whether the method {@link #update()} has been called since the last
    * time the vertices of this polygon have been modified.
    * <p>
    * Most operations with a polygon require the polygon to be up-to-date.
    * </p>
    */
   private boolean isUpToDate = false;

   /** Index of the vertex with the lowest x-coordinate. */
   private int minX_index = 0;
   /** Index of the vertex with the highest x-coordinate. */
   private int maxX_index = 0;
   /** Index of the vertex with the lowest y-coordinate. */
   private int minY_index = 0;
   /** Index of the vertex with the highest y-coordinate. */
   private int maxY_index = 0;
   /**
    * Index of the vertex with the lowest x-coordinate. If the lowest x-coordinate exists in more than
    * one vertex in the list, it is the index of the vertex with the highest y-coordinate out of the
    * candidates.
    * <p>
    * Note that the method {@link #update()} will always position this vertex at the index 0, so it
    * does not need to be updated.
    * </p>
    */
   private final int minXmaxY_index = 0;
   /**
    * Index of the vertex with the lowest x-coordinate. If the lowest x-coordinate exists in more than
    * one vertex in the list, it is the index of the vertex with the lowest y-coordinate out of the
    * candidates.
    */
   private int minXminY_index = 0;
   /**
    * Index of the vertex with the highest x-coordinate. If the highest x-coordinate exists in more
    * than one vertex in the list, it is the index of the vertex with the lowest y-coordinate out of
    * the candidates.
    */
   private int maxXminY_index = 0;
   /**
    * Index of the vertex with the highest x-coordinate. If the highest x-coordinate exists in more
    * than one vertex in the list, it is the index of the vertex with the highest y-coordinate out of
    * the candidates.
    */
   private int maxXmaxY_index = 0;

   /**
    * Creates an empty convex polygon.
    */
   public ConvexPolygon2D()
   {
      numberOfVertices = 0;
      update();
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of the points from the given
    * {@code vertices} list that are in the index range [0, {@code numberOfVertices}[.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the list. Only the points &in;
    *           [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the size
    *            of the given list of vertices.
    */
   public ConvexPolygon2D(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      setAndUpdate(vertices, numberOfVertices);
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of the points from the given
    * {@code vertices} list.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    */
   public ConvexPolygon2D(List<? extends Point2DReadOnly> vertices)
   {
      this(vertices, vertices.size());
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of the points from the given
    * {@code vertices} array that are in the index range [0, {@code numberOfVertices}[.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the size
    *            of the given array of vertices.
    */
   public ConvexPolygon2D(Point2DReadOnly[] vertices, int numberOfVertices)
   {
      setAndUpdate(vertices, numberOfVertices);
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of the points from the given
    * {@code vertices} array.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    */
   public ConvexPolygon2D(Point2DReadOnly[] vertices)
   {
      this(vertices, vertices.length);
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of the points from the given
    * {@code vertices} array that are in the index range [0, {@code numberOfVertices}[.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Each row
    *           contains one point whereas the (at least) two columns contains in order the coordinates
    *           x and y. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the size
    *            of the given array of vertices.
    */
   public ConvexPolygon2D(double[][] vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of the points from the given
    * {@code vertices} array.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Each row
    *           contains one point whereas the (at least) two columns contains in order the coordinates
    *           x and y. Not modified.
    */
   public ConvexPolygon2D(double[][] vertices)
   {
      this(vertices, vertices.length);
   }

   /**
    * Copy constructor.
    *
    * @param otherPolygon the other convex polygon to copy. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time the
    *            other polygon's vertices were edited.
    */
   public ConvexPolygon2D(ConvexPolygon2DReadOnly otherPolygon)
   {
      setAndUpdate(otherPolygon);
   }

   /**
    * Creates a new convex polygon by combining two other convex polygons. The result is the smallest
    * convex hull that contains both polygons.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param firstPolygon the first convex polygon to combine. Not modified.
    * @param secondPolygon the second convex polygon to combine. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time the
    *            other polygons' vertices were edited.
    */
   public ConvexPolygon2D(ConvexPolygon2DReadOnly firstPolygon, ConvexPolygon2DReadOnly secondPolygon)
   {
      setAndUpdate(firstPolygon, secondPolygon);
   }

   /**
    * Generates a random convex polygon given the maximum absolute coordinate value of its vertices and
    * the size of the point cloud from which it is generated.
    *
    * @deprecated Use {@link EuclidGeometryRandomTools#nextConvexPolygon2D(Random, double, int)}
    *             instead.
    *
    * @param random the random generator to use.
    * @param maxAbsoluteXY the maximum absolute value for each coordinate of the vertices.
    * @param numberOfPossiblePoints the size of the point cloud to generate that is used for computing
    *           the random convex polygon. The size of the resulting convex polygon will be less than
    *           {@code numberOfPossiblePoints}.
    * @return the random convex polygon.
    * @throws RuntimeException if {@code maxAbsoluteXY < 0}.
    */
   @Deprecated
   public static ConvexPolygon2D generateRandomConvexPolygon2d(Random random, double maxAbsoluteXY, int numberOfPossiblePoints)
   {
      List<Point2D> vertices = EuclidGeometryRandomTools.nextPointCloud2D(random, 0.0, maxAbsoluteXY, numberOfPossiblePoints);
      return new ConvexPolygon2D(vertices);
   }

   @Override
   public boolean isClockwiseOrdered()
   {
      return clockwiseOrdered;
   }

   /** {@inheritDoc} */
   public void clear()
   {
      numberOfVertices = 0;
      area = Double.NaN;
      centroid.set(Double.NaN, Double.NaN);
      isUpToDate = false;
   }

   /** {@inheritDoc} */
   public void clearAndUpdate()
   {
      clear();
      isUpToDate = true;
   }

   /** {@inheritDoc} */
   public void addVertex(double x, double y)
   {
      isUpToDate = false;
      setOrCreate(x, y, numberOfVertices);
      numberOfVertices++;
   }

   /** {@inheritDoc} */
   public void removeVertex(int indexOfVertexToRemove)
   {
      checkNonEmpty();
      checkIndexInBoundaries(indexOfVertexToRemove);

      if (indexOfVertexToRemove == numberOfVertices - 1)
      {
         numberOfVertices--;
         return;
      }
      isUpToDate = false;
      Collections.swap(clockwiseOrderedVertices, indexOfVertexToRemove, numberOfVertices - 1);
      numberOfVertices--;
   }

   /**
    * Method for internal use only.
    * <p>
    * Sets the {@code i}<sup>th</sup> point in {@link #clockwiseOrderedVertices} to the given point.
    * The list is extended if needed.
    * </p>
    *
    * @param x the x-coordinate of the point to copy. Not modified.
    * @param y the y-coordinate of the point to copy. Not modified.
    * @param i the position in the list {@link #clockwiseOrderedVertices} to copy the given point.
    */
   private void setOrCreate(double x, double y, int i)
   {
      while (i >= clockwiseOrderedVertices.size())
         clockwiseOrderedVertices.add(new Point2D());
      clockwiseOrderedVertices.get(i).set(x, y);
   }

   /** {@inheritDoc} */
   public void update()
   {
      if (isUpToDate)
         return;

      numberOfVertices = EuclidGeometryPolygonTools.inPlaceGiftWrapConvexHull2D(clockwiseOrderedVertices, numberOfVertices);
      isUpToDate = true;

      updateCentroidAndArea();
      updateBoundingBox();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(ConvexPolygon2DReadOnly)}.
    * <li>{@link #update()}.
    * </ol>
    *
    * @param other the other convex polygon to copy. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time the
    *            other polygon's vertices were edited.
    */
   @Override
   public void set(ConvexPolygon2D other)
   {
      setAndUpdate(other);
   }

   /**
    * Updates the bounding box properties.
    */
   public void updateBoundingBox()
   {
      minX_index = 0;
      maxX_index = 0;
      minY_index = 0;
      maxY_index = 0;
      minXminY_index = 0;
      maxXmaxY_index = 0;
      maxXminY_index = 0;

      if (!isEmpty())
      {
         Point2DReadOnly firstVertex = getVertex(0);
         double minX = firstVertex.getX();
         double minY = firstVertex.getY();
         double maxX = firstVertex.getX();
         double maxY = firstVertex.getY();

         Point2DReadOnly p;
         for (int i = 1; i < numberOfVertices; i++)
         {
            p = getVertex(i);

            if (p.getX() < minX)
            {
               minX = p.getX();
               minX_index = i;
               minXminY_index = i;
            }
            else if (p.getX() > maxX)
            {
               maxX = p.getX();
               maxX_index = i;
               maxXmaxY_index = i;
               maxXminY_index = i;
            }
            else if (p.getX() == getVertex(minXminY_index).getX() && p.getY() < getVertex(minXminY_index).getY())
            {
               minXminY_index = i;
            }
            else if (p.getX() == getVertex(maxXminY_index).getX()) // any case: getVertex(maxXmaxY_index).x == getVertex(maxXminY_index).x
            {
               if (p.getY() < getVertex(maxXminY_index).getY())
               {
                  maxXminY_index = i;
               }
               else if (p.getY() > getVertex(maxXmaxY_index).getY())
               {
                  maxXmaxY_index = i;
               }
            }

            if (p.getY() <= minY)
            {
               minY = p.getY();
               minY_index = i;
            }
            else if (p.getY() >= maxY)
            {
               maxY = p.getY();
               maxY_index = i;
            }
         }
         boundingBox.set(minX, minY, maxX, maxY);
      }
      else
      {
         boundingBox.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
      }
   }

   /**
    * Compute centroid and area of this polygon. Formula taken from
    * <a href= "http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/">here</a>.
    */
   public void updateCentroidAndArea()
   {
      area = EuclidGeometryPolygonTools.computeConvexPolyong2DArea(clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered, centroid);
   }

   /** {@inheritDoc} */
   @Override
   public double getArea()
   {
      checkIfUpToDate();
      return area;
   }

   /** {@inheritDoc} */
   @Override
   public Point2DReadOnly getCentroid()
   {
      checkIfUpToDate();
      return centroid;
   }

   /** {@inheritDoc} */
   @Override
   public BoundingBox2D getBoundingBox()
   {
      checkIfUpToDate();
      return boundingBox;
   }

   @Override
   public void setVertex(int index, double x, double y)
   {
      checkNonEmpty();
      checkIndexInBoundaries(index);
      clockwiseOrderedVertices.get(index).set(x, y);
   }

   /**
    * Gets the number of vertices composing this convex polygon.
    *
    * @return this polygon's size.
    */
   @Override
   public int getNumberOfVertices()
   {
      return numberOfVertices;
   }

   @Override
   public List<? extends Point2DReadOnly> getUnmodifiableVertexBuffer()
   {
      return unmodifiableVertexBuffer;
   }

   /** {@inheritDoc} */
   @Override
   public void applyTransform(Transform transform)
   {
      checkIfUpToDate();
      notifyVerticesChanged();

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = clockwiseOrderedVertices.get(i);
         vertex.applyTransform(transform);
      }
      update();
   }

   /** {@inheritDoc} */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      checkIfUpToDate();
      notifyVerticesChanged();

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = clockwiseOrderedVertices.get(i);
         vertex.applyInverseTransform(transform);
      }
      update();
   }

   /** {@inheritDoc} */
   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      checkIfUpToDate();
      notifyVerticesChanged();

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = clockwiseOrderedVertices.get(i);
         vertex.applyTransform(transform, false);
      }
      update();
   }

   /** {@inheritDoc} */
   @Override
   public int getMinXIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return minX_index;
   }

   /** {@inheritDoc} */
   @Override
   public int getMaxXIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return maxX_index;
   }

   /** {@inheritDoc} */
   @Override
   public int getMinYIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return minY_index;
   }

   /** {@inheritDoc} */
   @Override
   public int getMaxYIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return maxY_index;
   }

   /** {@inheritDoc} */
   @Override
   public int getMinXMaxYIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return minXmaxY_index;
   }

   /** {@inheritDoc} */
   @Override
   public int getMinXMinYIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return minXminY_index;
   }

   /** {@inheritDoc} */
   @Override
   public int getMaxXMaxYIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return maxXmaxY_index;
   }

   /** {@inheritDoc} */
   @Override
   public int getMaxXMinYIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return maxXminY_index;
   }

   /**
    * Tests whether this polygon has been updated via {@link #update()} since last time its vertices
    * have been modified.
    *
    * @return {@code true} if this polygon is up-to-date and operations can be used, {@code false}
    *         otherwise.
    */
   @Override
   public boolean isUpToDate()
   {
      return isUpToDate;
   }

   @Override
   public void notifyVerticesChanged()
   {
      isUpToDate = false;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof ConvexPolygon2DReadOnly)
         return ConvexPolygon2DBasics.super.equals((ConvexPolygon2DReadOnly) object);
      else
         return false;
   }

   /**
    * Tests on a per-component basis on every vertices if this convex polygon is equal to {@code other}
    * with the tolerance {@code epsilon}.
    * <p>
    * The method returns {@code false} if the two polygons have different size.
    * </p>
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two line segments are equal, {@code false} otherwise.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   @Override
   public boolean epsilonEquals(ConvexPolygon2D other, double epsilon)
   {
      return ConvexPolygon2DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two convex polygons are geometrically
    * similar.
    * <p>
    * This method performs the comparison on a per vertex basis while accounting for a possible shift
    * in the polygon indexing. For instance, two polygons that have the same vertices in clockwise or
    * counter-clockwise order, are considered geometrically equal even if they do not start with the
    * same vertex.
    * </p>
    *
    * @param other the convex polygon to compare to.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the convex polygons represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(ConvexPolygon2D other, double epsilon)
   {
      return ConvexPolygon2DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Creates and returns a representative {@code String} for this polygon.
    */
   @Override
   public String toString()
   {
      String ret = "";

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = clockwiseOrderedVertices.get(i);
         ret = ret + "(" + vertex.getX() + ", " + vertex.getY() + ")," + "\n";
      }

      return ret;
   }
}
