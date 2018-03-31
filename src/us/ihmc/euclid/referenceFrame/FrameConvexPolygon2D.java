package us.ihmc.euclid.referenceFrame;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class FrameConvexPolygon2D implements FrameConvexPolygon2DBasics, GeometryObject<FrameConvexPolygon2D>
{
   /**
    * Field for future expansion of {@code ConvexPolygon2d} to enable having the vertices in
    * clockwise or counter-clockwise ordered.
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
    * The vertices composing this polygon are located in the index range [0,
    * {@link #numberOfVertices}[ in this list.
    * </p>
    */
   private final List<FrameVertex2D> clockwiseOrderedVertices = new ArrayList<>();
   private final List<FixedFramePoint2DBasics> unmodifiableVertexBuffer = Collections.unmodifiableList(clockwiseOrderedVertices);
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
   private final FramePoint2DReadOnly frameCentroid = new FramePoint2DReadOnly()
   {
      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return centroid.getX();
      }

      @Override
      public double getY()
      {
         return centroid.getY();
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

   /** Index of the vertex with the lowest x-coordinate. */
   private int minX_index = 0;
   /** Index of the vertex with the highest x-coordinate. */
   private int maxX_index = 0;
   /** Index of the vertex with the lowest y-coordinate. */
   private int minY_index = 0;
   /** Index of the vertex with the highest y-coordinate. */
   private int maxY_index = 0;
   /**
    * Index of the vertex with the lowest x-coordinate. If the lowest x-coordinate exists in more
    * than one vertex in the list, it is the index of the vertex with the highest y-coordinate out
    * of the candidates.
    * <p>
    * Note that the method {@link #update()} will always position this vertex at the index 0, so it
    * does not need to be updated.
    * </p>
    */
   private final int minXmaxY_index = 0;
   /**
    * Index of the vertex with the lowest x-coordinate. If the lowest x-coordinate exists in more
    * than one vertex in the list, it is the index of the vertex with the lowest y-coordinate out of
    * the candidates.
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
    * than one vertex in the list, it is the index of the vertex with the highest y-coordinate out
    * of the candidates.
    */
   private int maxXmaxY_index = 0;

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

   public FrameConvexPolygon2D(ReferenceFrame referenceFrame, List<? extends Point2DReadOnly> vertices)
   {
      setIncludingFrameAndUpdate(referenceFrame, vertices, vertices.size());
   }

   public FrameConvexPolygon2D(ReferenceFrame referenceFrame, Point2DReadOnly[] vertices)
   {
      setIncludingFrameAndUpdate(referenceFrame, vertices, vertices.length);
   }

   public FrameConvexPolygon2D(List<? extends FramePoint2DReadOnly> frameVertices)
   {
      setIncludingFrameAndUpdate(frameVertices, frameVertices.size());
   }

   public FrameConvexPolygon2D(ReferenceFrame referenceFrame, ConvexPolygon2DReadOnly otherPolygon)
   {
      setIncludingFrameAndUpdate(referenceFrame, otherPolygon);
   }

   public FrameConvexPolygon2D(FrameConvexPolygon2DReadOnly other)
   {
      setIncludingFrameAndUpdate(other);
   }

   public FrameConvexPolygon2D(FrameConvexPolygon2DReadOnly firstPolygon, FrameConvexPolygon2DReadOnly secondPolygon)
   {
      setIncludingFrameAndUpdate(firstPolygon, secondPolygon);
   }

   @Override
   public void set(FrameConvexPolygon2D other)
   {
      setAndUpdate(other);
   }

   @Override
   public FixedFramePoint2DBasics getVertexUnsafe(int index)
   {
      checkNonEmpty();
      checkIndexInBoundaries(index);
      return clockwiseOrderedVertices.get(index);
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

      numberOfVertices = EuclidGeometryPolygonTools.inPlaceGiftWrapConvexHull2D(clockwiseOrderedVertices, numberOfVertices);
      isUpToDate = true;

      updateCentroidAndArea();
      updateBoundingBox();
   }

   @Override
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

   @Override
   public void updateCentroidAndArea()
   {
      area = EuclidGeometryPolygonTools.computeConvexPolyong2DArea(clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered, centroid);
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
      while (i >= clockwiseOrderedVertices.size())
         clockwiseOrderedVertices.add(new FrameVertex2D());
      clockwiseOrderedVertices.get(i).set(x, y);
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
      Collections.swap(clockwiseOrderedVertices, indexOfVertexToRemove, numberOfVertices - 1);
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
       * By overriding changeFrame, on the transformToDesiredFrame is being checked instead of
       * checking both referenceFrame.transformToRoot and desiredFrame.transformToRoot.
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
   public List<? extends FramePoint2DReadOnly> getUnmodifiableVertexBuffer()
   {
      return unmodifiableVertexBuffer;
   }

   @Override
   public FramePoint2DReadOnly getCentroid()
   {
      return frameCentroid;
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
   public BoundingBox2DReadOnly getBoundingBox()
   {
      return boundingBox;
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
