package us.ihmc.euclid.referenceFrame;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DBasics;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex3DSupplier;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
public class FrameConvexPolygon2D implements FrameConvexPolygon2DBasics, GeometryObject<FrameConvexPolygon2D>
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
   private final List<FrameVertex2D> vertexBuffer = new ArrayList<>();
   private final List<FixedFramePoint2DBasics> vertexBufferView = Collections.unmodifiableList(vertexBuffer);
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
   private final FixedFramePoint2DBasics centroid = new FixedFramePoint2DBasics()
   {
      private double x, y;

      @Override
      public void setX(double x)
      {
         this.x = x;
      };

      @Override
      public void setY(double y)
      {
         this.y = y;
      }

      @Override
      public double getX()
      {
         return x;
      }

      @Override
      public double getY()
      {
         return y;
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public String toString()
      {
         return EuclidCoreIOTools.getTuple2DString(this) + "-" + referenceFrame;
      }
   };
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
    * @param referenceFrame the initial reference frame for this polygon.
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
    * @param referenceFrame the initial reference frame for this polygon.
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
    * @param firstVertex2DSupplier the first supplier of vertices.
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
      FrameConvexPolygon2DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint2DBasics getVertexUnsafe(int index)
   {
      checkNonEmpty();
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
   }

   /** {@inheritDoc} */
   @Override
   public void clearAndUpdate()
   {
      clear();
      isUpToDate = true;
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

      updateCentroidAndArea();
      updateBoundingBox();
   }

   /** {@inheritDoc} */
   @Override
   public void updateCentroidAndArea()
   {
      area = EuclidGeometryPolygonTools.computeConvexPolyong2DArea(vertexBuffer, numberOfVertices, clockwiseOrdered, centroid);
   }

   /** {@inheritDoc} */
   @Override
   public void addVertex(double x, double y)
   {
      isUpToDate = false;
      setOrCreate(x, y, numberOfVertices);
      numberOfVertices++;
   }

   private void setOrCreate(double x, double y, int i)
   {
      while (i >= vertexBuffer.size())
         vertexBuffer.add(new FrameVertex2D());
      vertexBuffer.get(i).set(x, y);
   }

   /** {@inheritDoc} */
   @Override
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
   public FramePoint2DReadOnly getCentroid()
   {
      return centroid;
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
      return area;
   }

   /** {@inheritDoc} */
   @Override
   public BoundingBox2DBasics getBoundingBox()
   {
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

   /** {@inheritDoc} */
   @Override
   public boolean epsilonEquals(FrameConvexPolygon2D other, double epsilon)
   {
      return FrameConvexPolygon2DBasics.super.epsilonEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public boolean geometricallyEquals(FrameConvexPolygon2D other, double epsilon)
   {
      return FrameConvexPolygon2DBasics.super.geometricallyEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getConvexPolygon2DString(this) + "-" + referenceFrame;
   }

   private class FrameVertex2D implements FixedFramePoint2DBasics
   {
      private double x, y;

      @Override
      public void setX(double x)
      {
         this.x = x;
      }

      @Override
      public void setY(double y)
      {
         this.y = y;
      }

      @Override
      public double getX()
      {
         return x;
      }

      @Override
      public double getY()
      {
         return y;
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public String toString()
      {
         return EuclidCoreIOTools.getTuple2DString(this) + "-" + referenceFrame;
      }
   }
}
