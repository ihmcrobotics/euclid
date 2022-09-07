package us.ihmc.euclid.geometry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools.Convexity;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.AffineTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
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
public class ConvexPolygon2D implements ConvexPolygon2DBasics, Settable<ConvexPolygon2D>
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
    * It is updated in the method {@link #updateBoundingBox()} when calling {@link #getBoundingBox()}.
    * </p>
    */
   private final BoundingBox2D boundingBox = new BoundingBox2D();
   /**
    * The centroid of this polygon which is located at the center of mass of this polygon when
    * considered as a physical object with constant thickness and density.
    * <p>
    * It is updated in the method {@link #updateCentroidAndArea()} when calling {@link #getCentroid()}.
    * </p>
    */
   private final Point2D centroid = new Point2D();
   /**
    * The area of this convex polygon.
    * <p>
    * It is updated in the method {@link #updateCentroidAndArea()} when calling {@link #getArea()}
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
   private boolean boundingBoxDirty = true;
   private boolean areaCentroidDirty = true;

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
    * @param firstVertex2DSupplier  the first supplier of vertices.
    * @param secondVertex2DSupplier the second supplier of vertices.
    * @see #set(Vertex2DSupplier, Vertex2DSupplier)
    */
   public ConvexPolygon2D(Vertex2DSupplier firstVertex2DSupplier, Vertex2DSupplier secondVertex2DSupplier)
   {
      set(firstVertex2DSupplier, secondVertex2DSupplier);
   }

   @Override
   public boolean isClockwiseOrdered()
   {
      return clockwiseOrdered;
   }

   /** {@inheritDoc} */
   @Override
   public void clear()
   {
      numberOfVertices = 0;
      area = Double.NaN;
      centroid.setToNaN();
      boundingBox.setToNaN();
      isUpToDate = false;
      boundingBoxDirty = true;
      areaCentroidDirty = true;
   }

   /** {@inheritDoc} */
   @Override
   public void clearAndUpdate()
   {
      clear();
      isUpToDate = true;
      boundingBoxDirty = false;
      areaCentroidDirty = false;
   }

   /** {@inheritDoc} */
   @Override
   public void addVertex(double x, double y)
   {
      isUpToDate = false;
      setOrCreate(numberOfVertices, x, y);
      numberOfVertices++;
   }

   /** {@inheritDoc} */
   @Override
   public void removeVertex(int indexOfVertexToRemove)
   {
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
    * @param i the position in the list {@link #vertexBuffer} to copy the given point.
    * @param x the x-coordinate of the point to copy. Not modified.
    * @param y the y-coordinate of the point to copy. Not modified.
    */
   private void setOrCreate(int i, double x, double y)
   {
      while (i >= vertexBuffer.size())
         vertexBuffer.add(new Point2D());
      vertexBuffer.get(i).set(x, y);
   }

   /** {@inheritDoc} */
   @Override
   public void update()
   {
      if (isUpToDate)
         return;

      numberOfVertices = EuclidGeometryPolygonTools.inPlaceGiftWrapConvexHull2D(vertexBuffer, numberOfVertices);
      isUpToDate = true;
      boundingBoxDirty = true;
      areaCentroidDirty = true;
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
    *                                  other polygon's vertices were edited.
    */
   @Override
   public void set(ConvexPolygon2D other)
   {
      if (clockwiseOrdered != other.clockwiseOrdered)
      {
         // TODO For now relying on the expensive method to ensure consistent ordering.
         ConvexPolygon2DBasics.super.set(other);
         return;
      }

      numberOfVertices = other.numberOfVertices;

      for (int i = 0; i < other.numberOfVertices; i++)
      {
         Point2D otherVertex = other.vertexBuffer.get(i);
         setOrCreate(i, otherVertex.getX(), otherVertex.getY());
      }
      boundingBox.set(other.boundingBox);
      centroid.set(other.centroid);
      area = other.area;
      isUpToDate = other.isUpToDate;
      boundingBoxDirty = other.boundingBoxDirty;
      areaCentroidDirty = other.areaCentroidDirty;
   }

   @Override
   public void set(Vertex2DSupplier vertex2DSupplier)
   {
      if (vertex2DSupplier instanceof ConvexPolygon2D)
      {
         set((ConvexPolygon2D) vertex2DSupplier);
      }
      else if (vertex2DSupplier instanceof ConvexPolygon2DReadOnly)
      {
         ConvexPolygon2DReadOnly other = (ConvexPolygon2DReadOnly) vertex2DSupplier;

         if (clockwiseOrdered != other.isClockwiseOrdered())
         {
            // TODO For now relying on the expensive method to ensure consistent ordering.
            ConvexPolygon2DBasics.super.set(vertex2DSupplier);
            return;
         }

         clear();
         numberOfVertices = other.getNumberOfVertices();

         for (int i = 0; i < numberOfVertices; i++)
         {
            Point2DReadOnly otherVertex = other.getVertexUnsafe(i);
            setOrCreate(i, otherVertex.getX(), otherVertex.getY());
         }

         if (other.isUpToDate())
         {
            isUpToDate = true;
            boundingBoxDirty = true;
            areaCentroidDirty = true;
         }
      }
      else
      {
         ConvexPolygon2DBasics.super.set(vertex2DSupplier);
      }
   }

   /**
    * Compute centroid and area of this polygon. Formula taken from
    * <a href= "http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/">here</a>.
    */
   private void updateCentroidAndArea()
   {
      if (areaCentroidDirty)
      {
         areaCentroidDirty = false;
         area = EuclidGeometryPolygonTools.computeConvexPolygon2DArea(vertexBuffer, numberOfVertices, clockwiseOrdered, centroid);
      }
   }

   /**
    * Updates the bounding box properties.
    */
   private void updateBoundingBox()
   {
      if (boundingBoxDirty)
      {
         boundingBoxDirty = false;
         boundingBox.setToNaN();
         boundingBox.updateToIncludePoints(this);
      }
   }

   @Override
   public Point2DBasics getVertexUnsafe(int index)
   {
      checkIndexInBoundaries(index);
      return vertexBuffer.get(index);
   }

   /** {@inheritDoc} */
   @Override
   public double getArea()
   {
      checkIfUpToDate();
      updateCentroidAndArea();
      return area;
   }

   /** {@inheritDoc} */
   @Override
   public Point2DReadOnly getCentroid()
   {
      checkIfUpToDate();
      updateCentroidAndArea();
      return centroid;
   }

   /** {@inheritDoc} */
   @Override
   public BoundingBox2DReadOnly getBoundingBox()
   {
      checkIfUpToDate();
      updateBoundingBox();
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
   public void translate(double x, double y)
   {
      checkIfUpToDate();

      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         getVertexUnsafe(i).add(x, y);
      }

      if (!boundingBoxDirty)
      {
         boundingBox.getMinPoint().add(x, y);
         boundingBox.getMaxPoint().add(x, y);
      }
      if (!areaCentroidDirty)
      {
         centroid.add(x, y);
      }
   }

   @Override
   public void applyTransform(Transform transform, boolean checkIfTransformInXYPlane)
   {
      checkIfUpToDate();

      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         getVertexUnsafe(i).applyTransform(transform, checkIfTransformInXYPlane);
      }

      postTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform, boolean checkIfTransformInXYPlane)
   {
      checkIfUpToDate();

      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         getVertexUnsafe(i).applyInverseTransform(transform, checkIfTransformInXYPlane);
      }

      postTransform(transform);
   }

   private void postTransform(Transform transform)
   {
      if (numberOfVertices <= 3)
      { // It's real cheap to update when dealing with few vertices.
         notifyVerticesChanged();
         update();
         return;
      }

      boolean updateVertices = true;

      if (transform instanceof RigidBodyTransformReadOnly)
      {
         RigidBodyTransformReadOnly rbTransform = (RigidBodyTransformReadOnly) transform;
         updateVertices = rbTransform.hasRotation();
      }
      else if (transform instanceof AffineTransformReadOnly)
      {
         AffineTransformReadOnly aTransform = (AffineTransformReadOnly) transform;
         updateVertices = aTransform.hasLinearTransform();
      }

      if (updateVertices)
      {
         // Testing ordering by looking at the convexity
         Convexity convexity = null;

         for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
         {
            if (convexity == null)
               convexity = EuclidGeometryPolygonTools.polygon2DConvexityAtVertex(vertexIndex, vertexBuffer, vertexIndex, clockwiseOrdered);
            if (convexity != null)
               break;
         }

         if (convexity == Convexity.CONCAVE)
         { // The polygon got flipped, need to reverse the order to preserve the order.
            EuclidCoreTools.reverse(vertexBuffer, 0, numberOfVertices);
         }

         // Shifting vertices around to ensure the first vertex is min-x (and max-y if multiple min-xs)
         int minXMaxYVertexIndex = EuclidGeometryPolygonTools.findMinXMaxYVertexIndex(vertexBuffer, numberOfVertices);
         EuclidCoreTools.rotate(vertexBuffer, 0, numberOfVertices, -minXMaxYVertexIndex);
      }

      // Being lazy, could transform these too.
      boundingBoxDirty = true;
      areaCentroidDirty = true;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof ConvexPolygon2DReadOnly)
         return equals((EuclidGeometry) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(EuclidHashCodeTools.addToHashCode(Boolean.hashCode(clockwiseOrdered), vertexBufferView));
   }

   /**
    * Creates and returns a representative {@code String} for this polygon.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
