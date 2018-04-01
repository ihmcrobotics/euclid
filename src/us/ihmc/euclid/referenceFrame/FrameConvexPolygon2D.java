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

   private ReferenceFrame referenceFrame;

   private final Point3D vertex3D = new Point3D();

   public FrameConvexPolygon2D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameConvexPolygon2D(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      clearAndUpdate();
   }

   public FrameConvexPolygon2D(ReferenceFrame referenceFrame, Vertex2DSupplier vertex2DSupplier)
   {
      setIncludingFrame(referenceFrame, vertex2DSupplier);
   }

   public FrameConvexPolygon2D(ReferenceFrame referenceFrame, Vertex3DSupplier vertex3DSupplier)
   {
      setIncludingFrame(referenceFrame, vertex3DSupplier);
   }

   public FrameConvexPolygon2D(FrameVertex2DSupplier vertex2DSupplier)
   {
      setIncludingFrame(vertex2DSupplier);
   }

   public FrameConvexPolygon2D(FrameVertex3DSupplier vertex3DSupplier)
   {
      setIncludingFrame(vertex3DSupplier);
   }

   public FrameConvexPolygon2D(FrameVertex2DSupplier firstVertex2DSupplier, FrameVertex2DSupplier secondVertex2DSupplier)
   {
      setIncludingFrame(firstVertex2DSupplier, secondVertex2DSupplier);
   }

   @Override
   public void set(FrameConvexPolygon2D other)
   {
      FrameConvexPolygon2DBasics.super.set(other);
   }

   @Override
   public FixedFramePoint2DBasics getVertexUnsafe(int index)
   {
      checkNonEmpty();
      checkIndexInBoundaries(index);
      return vertexBuffer.get(index);
   }

   @Override
   public void notifyVerticesChanged()
   {
      isUpToDate = false;
   }

   @Override
   public void clear()
   {
      numberOfVertices = 0;
      area = Double.NaN;
      centroid.setToNaN();
      isUpToDate = false;
   }

   @Override
   public void clearAndUpdate()
   {
      clear();
      isUpToDate = true;
   }

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

   @Override
   public void updateCentroidAndArea()
   {
      area = EuclidGeometryPolygonTools.computeConvexPolyong2DArea(vertexBuffer, numberOfVertices, clockwiseOrdered, centroid);
   }

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

   @Override
   public List<? extends FramePoint2DReadOnly> getVertexBufferView()
   {
      return vertexBufferView;
   }

   @Override
   public FramePoint2DReadOnly getCentroid()
   {
      return centroid;
   }

   @Override
   public boolean isClockwiseOrdered()
   {
      return clockwiseOrdered;
   }

   @Override
   public boolean isUpToDate()
   {
      return isUpToDate;
   }

   @Override
   public int getNumberOfVertices()
   {
      return numberOfVertices;
   }

   @Override
   public double getArea()
   {
      return area;
   }

   @Override
   public BoundingBox2DBasics getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public boolean epsilonEquals(FrameConvexPolygon2D other, double epsilon)
   {
      return FrameConvexPolygon2DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(FrameConvexPolygon2D other, double epsilon)
   {
      return FrameConvexPolygon2DBasics.super.geometricallyEquals(other, epsilon);
   }

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
