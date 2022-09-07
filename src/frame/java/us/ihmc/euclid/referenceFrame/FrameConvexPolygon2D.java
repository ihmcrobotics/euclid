package us.ihmc.euclid.referenceFrame;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools.Convexity;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBoundingBox2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex3DSupplier;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.AffineTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Describes a planar convex polygon defined in the XY-plane and that is expressed in a changeable
 * reference frame.
 * <p>
 * The vertices of a convex polygon are clockwise ordered and are all different.
 * </p>
 * <p>
 * This implementation of convex polygon is designed for garbage free operations.
 * </p>
 */
public class FrameConvexPolygon2D implements FrameConvexPolygon2DBasics, Settable<FrameConvexPolygon2D>
{
   /**
    * Field for future expansion of {@code ConvexPolygon2d} to enable having the vertices in clockwise
    * or counter-clockwise ordered.
    */
   private final boolean clockwiseOrdered = true;
   /** Rigid-body transform used to perform garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();
   /**
    * The current size or number of vertices for this convex polygon.
    */
   private int numberOfVertices = 0;
   /**
    * The internal memory of {@code FrameConvexPolygon2d}.
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
   private final List<FixedFramePoint2DBasics> vertexBuffer = new ArrayList<>();
   private final List<FixedFramePoint2DBasics> vertexBufferView = Collections.unmodifiableList(vertexBuffer);

   /**
    * The smallest axis-aligned bounding box that contains all this polygon's vertices.
    * <p>
    * It is updated in the method {@link #updateBoundingBox()} when calling {@link #getBoundingBox()}.
    * </p>
    */
   private final FixedFrameBoundingBox2DBasics boundingBox = EuclidFrameFactories.newFixedFrameBoundingBox2DBasics(this);
   /**
    * The centroid of this polygon which is located at the center of mass of this polygon when
    * considered as a physical object with constant thickness and density.
    * <p>
    * It is updated in the method {@link #updateCentroidAndArea()} when {@link #getCentroid()}
    * </p>
    */
   private final FixedFramePoint2DBasics centroid = EuclidFrameFactories.newFixedFramePoint2DBasics(this);
   /**
    * The area of this convex polygon.
    * <p>
    * It is updated in the method {@link #updateCentroidAndArea()} when {@link #getArea()}.
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
   /** The reference frame in which this polygon is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** Vertex to store intermediate results to allow garbage free operations. */
   private final Point3D vertex3D = new Point3D();

   /**
    * Creates an empty convex polygon in world frame.
    */
   public FrameConvexPolygon2D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates an empty convex polygon in the given reference frame.
    *
    * @param referenceFrame the initial reference frame for this polygon.
    */
   public FrameConvexPolygon2D(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      clearAndUpdate();
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of all the points provided
    * by the supplier.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param referenceFrame   the initial reference frame for this polygon.
    * @param vertex2DSupplier the supplier of vertices.
    * @see #setIncludingFrame(ReferenceFrame, Vertex2DSupplier)
    */
   public FrameConvexPolygon2D(ReferenceFrame referenceFrame, Vertex2DSupplier vertex2DSupplier)
   {
      setIncludingFrame(referenceFrame, vertex2DSupplier);
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of all the points provided
    * by the supplier.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param referenceFrame   the initial reference frame for this polygon.
    * @param vertex3DSupplier the supplier of vertices.
    * @see #setIncludingFrame(ReferenceFrame, Vertex3DSupplier)
    */
   public FrameConvexPolygon2D(ReferenceFrame referenceFrame, Vertex3DSupplier vertex3DSupplier)
   {
      setIncludingFrame(referenceFrame, vertex3DSupplier);
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of all the points provided
    * by the supplier.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param frameVertex2DSupplier the supplier of vertices.
    * @see #setIncludingFrame(FrameVertex2DSupplier)
    */
   public FrameConvexPolygon2D(FrameVertex2DSupplier frameVertex2DSupplier)
   {
      setIncludingFrame(frameVertex2DSupplier);
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of all the points provided
    * by the supplier.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param frameVertex3DSupplier the supplier of vertices.
    * @see #setIncludingFrame(FrameVertex3DSupplier)
    */
   public FrameConvexPolygon2D(FrameVertex3DSupplier frameVertex3DSupplier)
   {
      setIncludingFrame(frameVertex3DSupplier);
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
    * @see #setIncludingFrame(FrameVertex2DSupplier, FrameVertex2DSupplier)
    */
   public FrameConvexPolygon2D(FrameVertex2DSupplier firstVertex2DSupplier, FrameVertex2DSupplier secondVertex2DSupplier)
   {
      setIncludingFrame(firstVertex2DSupplier, secondVertex2DSupplier);
   }

   /**
    * {@inheritDoc}
    *
    * @see FrameConvexPolygon2D#set(FrameVertex2DSupplier)
    */
   @Override
   public void set(FrameConvexPolygon2D other)
   {
      if (clockwiseOrdered != other.clockwiseOrdered)
      {
         // TODO For now relying on the expensive method to ensure consistent ordering.
         FrameConvexPolygon2DBasics.super.set(other);
         return;
      }

      checkReferenceFrameMatch(other);

      numberOfVertices = other.numberOfVertices;

      for (int i = 0; i < other.numberOfVertices; i++)
      {
         FixedFramePoint2DBasics otherVertex = other.vertexBuffer.get(i);
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
      if (vertex2DSupplier instanceof ConvexPolygon2DReadOnly)
      {
         ConvexPolygon2DReadOnly other = (ConvexPolygon2DReadOnly) vertex2DSupplier;

         if (clockwiseOrdered != other.isClockwiseOrdered())
         {
            // TODO For now relying on the expensive method to ensure consistent ordering.
            FrameConvexPolygon2DBasics.super.set(vertex2DSupplier);
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
         FrameConvexPolygon2DBasics.super.set(vertex2DSupplier);
      }
   }

   @Override
   public void set(FrameVertex2DSupplier frameVertex2DSupplier)
   {
      if (frameVertex2DSupplier instanceof FrameConvexPolygon2D)
      {
         set((FrameConvexPolygon2D) frameVertex2DSupplier);
      }
      else if (frameVertex2DSupplier instanceof FrameConvexPolygon2DReadOnly)
      {
         FrameConvexPolygon2DReadOnly other = (FrameConvexPolygon2DReadOnly) frameVertex2DSupplier;

         if (clockwiseOrdered != other.isClockwiseOrdered())
         {
            // TODO For now relying on the expensive method to ensure consistent ordering.
            FrameConvexPolygon2DBasics.super.set(frameVertex2DSupplier);
            return;
         }

         clear();
         numberOfVertices = other.getNumberOfVertices();

         for (int i = 0; i < numberOfVertices; i++)
         {
            FramePoint2DReadOnly otherVertex = other.getVertexUnsafe(i);
            checkReferenceFrameMatch(otherVertex);
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
         FrameConvexPolygon2DBasics.super.set(frameVertex2DSupplier);
      }
   }

   @Override
   public void setMatchingFrame(FrameVertex2DSupplier frameVertex2DSupplier, boolean checkIfTransformInXYPlane)
   {
      set((Vertex2DSupplier) frameVertex2DSupplier);

      if (frameVertex2DSupplier.getReferenceFrame() != referenceFrame)
      {
         frameVertex2DSupplier.getReferenceFrame().getTransformToDesiredFrame(transformToDesiredFrame, referenceFrame);
         applyTransform(transformToDesiredFrame, checkIfTransformInXYPlane);
      }
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint2DBasics getVertexUnsafe(int index)
   {
      checkIndexInBoundaries(index);
      return vertexBuffer.get(index);
   }

   /** {@inheritDoc} */
   @Override
   public void notifyVerticesChanged()
   {
      isUpToDate = false;
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
   public void addVertexMatchingFrame(ReferenceFrame referenceFrame, Point2DReadOnly vertex, boolean checkIfTransformInXYPlane)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (getReferenceFrame() == referenceFrame)
      {
         addVertex(vertex);
      }
      else
      {
         referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, getReferenceFrame());
         addVertex(vertex);
         getVertexUnsafe(getNumberOfVertices() - 1).applyTransform(transformToDesiredFrame, checkIfTransformInXYPlane);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void addVertexMatchingFrame(ReferenceFrame referenceFrame, Point3DReadOnly vertex)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (getReferenceFrame() == referenceFrame)
      {
         addVertex(vertex);
      }
      else
      {
         referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, getReferenceFrame());
         transformToDesiredFrame.transform(vertex, vertex3D);
         addVertex(vertex3D);
      }
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
      checkIfUpToDate();
      if (boundingBoxDirty)
      {
         boundingBoxDirty = false;
         boundingBox.setToNaN();
         boundingBox.updateToIncludePoints(this);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void addVertex(double x, double y)
   {
      isUpToDate = false;
      setOrCreate(numberOfVertices, x, y);
      numberOfVertices++;
   }

   private void setOrCreate(int i, double x, double y)
   {
      while (i >= vertexBuffer.size())
         vertexBuffer.add(EuclidFrameFactories.newFixedFramePoint2DBasics(this));
      vertexBuffer.get(i).set(x, y);
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

   /** {@inheritDoc} */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == referenceFrame)
         return;

      /*
       * By overriding changeFrame, on the transformToDesiredFrame is being checked instead of checking
       * both referenceFrame.transformToRoot and desiredFrame.transformToRoot.
       */
      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame);
      referenceFrame = desiredFrame;
   }

   /** {@inheritDoc} */
   @Override
   public final void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame, false);
      referenceFrame = desiredFrame;
   }

   /** {@inheritDoc} */
   @Override
   public List<? extends FramePoint2DReadOnly> getVertexBufferView()
   {
      return vertexBufferView;
   }

   /** {@inheritDoc} */
   @Override
   public boolean isClockwiseOrdered()
   {
      return clockwiseOrdered;
   }

   /** {@inheritDoc} */
   @Override
   public boolean isUpToDate()
   {
      return isUpToDate;
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfVertices()
   {
      return numberOfVertices;
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
   public FramePoint2DReadOnly getCentroid()
   {
      checkIfUpToDate();
      updateCentroidAndArea();
      return centroid;
   }

   /** {@inheritDoc} */
   @Override
   public FrameBoundingBox2DReadOnly getBoundingBox()
   {
      checkIfUpToDate();
      updateBoundingBox();
      return boundingBox;
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
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
      if (object instanceof FrameConvexPolygon2DReadOnly)
         return equals((EuclidFrameGeometry) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.addToHashCode(Boolean.hashCode(clockwiseOrdered), vertexBufferView);
      bits = EuclidHashCodeTools.addToHashCode(bits, referenceFrame);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   /** {@inheritDoc} */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
