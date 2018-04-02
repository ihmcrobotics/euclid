package us.ihmc.euclid.geometry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import us.ihmc.euclid.geometry.exceptions.EmptyPolygonException;
import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * Describes a planar convex polygon defined in the XY-plane.
 * <p>
 * The vertices of a convex polygon are clockwise ordered and are all different.
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
   private final List<Point2D> vertexBuffer = new ArrayList<>();
   private final List<Point2D> vertexBufferView = Collections.unmodifiableList(vertexBuffer);
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

   /**
    * Creates an empty convex polygon.
    */
   public ConvexPolygon2D()
   {
      numberOfVertices = 0;
      update();
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of all the points provided
    * by the supplier.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param vertex2DSupplier the supplier of vertices.
    * @see #set(Vertex2DSupplier)
    */
   public ConvexPolygon2D(Vertex2DSupplier vertex2DSupplier)
   {
      set(vertex2DSupplier);
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of all the points provided
    * by the supplier.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param vertex3DSupplier the supplier of vertices.
    * @see #set(Vertex2DSupplier)
    */
   public ConvexPolygon2D(Vertex3DSupplier vertex3DSupplier)
   {
      set(vertex3DSupplier);
   }

   /**
    * Creates a new convex polygon by combining the vertices from two suppliers. The result is the
    * smallest convex hull that contains all the vertices provided by the two suppliers.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param firstVertex2DSupplier the first supplier of vertices.
    * @param secondVertex2DSupplier the second supplier of vertices.
    * @see #set(Vertex2DSupplier, Vertex2DSupplier)
    */
   public ConvexPolygon2D(Vertex2DSupplier firstVertex2DSupplier, Vertex2DSupplier secondVertex2DSupplier)
   {
      set(firstVertex2DSupplier, secondVertex2DSupplier);
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
      return new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));
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
      centroid.setToNaN();
      boundingBox.setToNaN();
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
      Collections.swap(vertexBuffer, indexOfVertexToRemove, numberOfVertices - 1);
      numberOfVertices--;
   }

   /**
    * Method for internal use only.
    * <p>
    * Sets the {@code i}<sup>th</sup> point in {@link #vertexBuffer} to the given point. The list is
    * extended if needed.
    * </p>
    *
    * @param x the x-coordinate of the point to copy. Not modified.
    * @param y the y-coordinate of the point to copy. Not modified.
    * @param i the position in the list {@link #vertexBuffer} to copy the given point.
    */
   private void setOrCreate(double x, double y, int i)
   {
      while (i >= vertexBuffer.size())
         vertexBuffer.add(new Point2D());
      vertexBuffer.get(i).set(x, y);
   }

   /** {@inheritDoc} */
   public void update()
   {
      if (isUpToDate)
         return;

      numberOfVertices = EuclidGeometryPolygonTools.inPlaceGiftWrapConvexHull2D(vertexBuffer, numberOfVertices);
      isUpToDate = true;

      updateCentroidAndArea();
      updateBoundingBox();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(Vertex2DSupplier)}.
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
      ConvexPolygon2DBasics.super.set(other);
   }

   /**
    * Compute centroid and area of this polygon. Formula taken from
    * <a href= "http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/">here</a>.
    */
   public void updateCentroidAndArea()
   {
      area = EuclidGeometryPolygonTools.computeConvexPolyong2DArea(vertexBuffer, numberOfVertices, clockwiseOrdered, centroid);
   }

   @Override
   public Point2DBasics getVertexUnsafe(int index)
   {
      checkNonEmpty();
      checkIndexInBoundaries(index);
      return vertexBuffer.get(index);
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
   public List<? extends Point2DReadOnly> getVertexBufferView()
   {
      return vertexBufferView;
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
      return EuclidGeometryIOTools.getConvexPolygon2DString(this);
   }
}
