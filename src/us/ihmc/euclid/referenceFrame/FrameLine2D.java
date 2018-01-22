package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public class FrameLine2D implements FrameLine2DBasics, GeometryObject<FrameLine2D>
{
   private ReferenceFrame referenceFrame;
   /** The line. */
   private final Line2D line = new Line2D();
   /** Rigid-body transform used to perform garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   private final FixedFramePoint2DBasics point = new FixedFramePoint2DBasics()
   {
      @Override
      public void setX(double x)
      {
         line.getPoint().setX(x);
      }

      @Override
      public void setY(double y)
      {
         line.getPoint().setY(y);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return line.getPointX();
      }

      @Override
      public double getY()
      {
         return line.getPointY();
      }
   };

   private final FixedFrameVector2DBasics direction = new FixedFrameVector2DBasics()
   {
      @Override
      public void setX(double x)
      {
         line.getDirection().setX(x);
      }

      @Override
      public void setY(double y)
      {
         line.getDirection().setY(y);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return line.getDirectionX();
      }

      @Override
      public double getY()
      {
         return line.getDirectionY();
      }
   };

   public FrameLine2D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   public FrameLine2D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FrameLine2D(Line2DReadOnly line2D)
   {
      this(ReferenceFrame.getWorldFrame(), line2D);
   }

   public FrameLine2D(ReferenceFrame referenceFrame, Line2DReadOnly line2DReadOnly)
   {
      setIncludingFrame(referenceFrame, line2DReadOnly);
   }

   public FrameLine2D(ReferenceFrame referenceFrame, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      setIncludingFrame(referenceFrame, firstPointOnLine, secondPointOnLine);
   }
   
   public FrameLine2D(ReferenceFrame referenceFrame, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      setIncludingFrame(referenceFrame, pointOnLine, lineDirection);
   }

   public FrameLine2D(LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      this(ReferenceFrame.getWorldFrame(), lineSegment2DReadOnly);
   }

   public FrameLine2D(ReferenceFrame referenceFrame, LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      setIncludingFrame(referenceFrame, lineSegment2DReadOnly);
   }

   public FrameLine2D(FrameLine2DReadOnly other)
   {
      setIncludingFrame(other);
   }
   
   public FrameLine2D(FrameLineSegment2DReadOnly frameLineSegment2DReadOnly)
   {
      setIncludingFrame(frameLineSegment2DReadOnly);
   }

   public FrameLine2D(FramePoint2DReadOnly firstPointOnLine, FramePoint2DReadOnly secondPointOnLine)
   {
      setIncludingFrame(firstPointOnLine, secondPointOnLine);
   }
   
   public FrameLine2D(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      setIncludingFrame(pointOnLine, lineDirection);
   }

   @Override
   public void set(FrameLine2D other)
   {
      FrameLine2DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint2DBasics getPoint()
   {
      return point;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector2DBasics getDirection()
   {
      return direction;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame, false);
      referenceFrame = desiredFrame;
   }

   /**
    * Tests on a per-component basis on the point and vector if this line is equal to {@code other}
    * with the tolerance {@code epsilon}. This method will return {@code false} if the two lines are
    * physically the same but either the point or vector of each line is different. For instance, if
    * {@code this.point == other.point} and {@code this.direction == - other.direction}, the two
    * lines are physically the same but this method returns {@code false}.
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two lines are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameLine2D other, double epsilon)
   {
      return FrameLine2DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two lines are geometrically
    * similar.
    * <p>
    * Two lines are considered geometrically equal is they are collinear, pointing toward the same
    * or opposite direction.
    * </p>
    *
    * @param other the line to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two lines represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   @Override
   public boolean geometricallyEquals(FrameLine2D other, double epsilon)
   {
      return FrameLine2DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(FrameLine2DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param obj the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((FrameLine2DReadOnly) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Provides a {@code String} representation of this frame line 2D as follows:<br>
    * Line 3D: point = (x, y), direction = (x, y)-worldFrame
    *
    * @return the {@code String} representing this line 2D.
    */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getLine2DString(this) + "-" + referenceFrame;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this line 2D.
    *
    * @return the hash code value for this line 2D.
    */
   @Override
   public int hashCode()
   {
      return line.hashCode();
   }
}
