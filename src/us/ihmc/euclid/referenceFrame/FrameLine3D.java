package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * {@code FrameLine3D} is a 3D line expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Line3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameLine3D}. This allows, for instance, to enforce, at runtime, that operations on lines
 * occur in the same coordinate system. Also, via the method {@link #changeFrame(ReferenceFrame)},
 * one can easily calculates the value of a point in different reference frames.
 * </p>
 * <p>
 * Because a {@code FrameLine3D} extends {@code Line3DBasics}, it is compatible with methods only
 * requiring {@code Line3DBasics}. However, these methods do NOT assert that the operation occur in
 * the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameLine3D}.
 * </p>
 */
public class FrameLine3D implements FrameLine3DBasics, GeometryObject<FrameLine3D>
{
   /** The reference frame in which this line is expressed. */
   private ReferenceFrame referenceFrame;
   /** The line. */
   private final Line3D line = new Line3D();

   private final FixedFramePoint3DBasics point = new FixedFramePoint3DBasics()
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
      public void setZ(double z)
      {
         line.getPoint().setZ(z);
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

      @Override
      public double getZ()
      {
         return line.getPointZ();
      }
   };

   private final FixedFrameVector3DBasics direction = new FixedFrameVector3DBasics()
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
      public void setZ(double z)
      {
         line.getDirection().setZ(z);
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

      @Override
      public double getZ()
      {
         return line.getDirectionZ();
      }
   };

   /**
    * Default constructor that initializes both {@link #point} and {@link #direction} to zero and the
    * reference frame to {@code ReferenceFrame.getWorldFrame()}.
    */
   public FrameLine3D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new line and initializes both {@link #point} and {@link #direction} to zero and the
    * reference frame to the given {@code referenceFrame}.
    *
    * @param referenceFrame the initial reference frame for this line.
    */
   public FrameLine3D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new line, initializes its point and direction from the given line and its reference
    * frame to {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param line2DReadOnly the line used to initialize the point and direction of this. Not modified.
    */
   public FrameLine3D(Line2DReadOnly line2DReadOnly)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), line2DReadOnly);
   }

   /**
    * Creates a new line, initializes its point and direction from the given line and its reference
    * frame to {@code referenceFrame}.
    *
    * @param referenceFrame the initial reference frame for this line.
    * @param line2DReadOnly the line used to initialize the point and direction of this. Not modified.
    */
   public FrameLine3D(ReferenceFrame referenceFrame, Line2DReadOnly line2DReadOnly)
   {
      setIncludingFrame(referenceFrame, line2DReadOnly);
   }

   /**
    * Creates a new line, initializes its point and direction from the given line and its reference
    * frame to {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param line3DReadOnly the line used to initialize the point and direction of this. Not modified.
    */
   public FrameLine3D(Line3DReadOnly line3DReadOnly)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), line3DReadOnly);
   }

   /**
    * Creates a new line, initializes its point and direction from the given line and its reference
    * frame to {@code referenceFrame}.
    *
    * @param referenceFrame the initial reference frame for this line.
    * @param line3DReadOnly the line used to initialize the point and direction of this. Not modified.
    */
   public FrameLine3D(ReferenceFrame referenceFrame, Line3DReadOnly line3DReadOnly)
   {
      setIncludingFrame(referenceFrame, line3DReadOnly);
   }

   /**
    * Creates a new line, initializes it to go through the two given points in the given frame.
    *
    * @param referenceFrame the initial reference frame for this line.
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    */
   public FrameLine3D(ReferenceFrame referenceFrame, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      setIncludingFrame(referenceFrame, firstPointOnLine, secondPointOnLine);
   }

   /**
    * Creates a new line, initializes it using the given point and direction in the given frame.
    *
    * @param referenceFrame the initial reference frame for this line.
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    */
   public FrameLine3D(ReferenceFrame referenceFrame, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      setIncludingFrame(referenceFrame, pointOnLine, lineDirection);
   }

   /**
    * Creates a new line and initializes it to go through the endpoints of the given line segment.
    * <p>
    * The reference frame is initialized to {@code ReferenceFrame.getWorldFrame()}.
    * </p>
    *
    * @param lineSegment2DReadOnly the line segment to copy. Not modified.
    */
   public FrameLine3D(LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), lineSegment2DReadOnly);
   }

   /**
    * Creates a new line and initializes it to go through the endpoints of the given line segment.
    * <p>
    * The reference frame is initialized to {@code ReferenceFrame.getWorldFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the given line segment is expressed. The
    *           initial frame for this frame line.
    * @param lineSegment2DReadOnly the line segment to copy. Not modified.
    */
   public FrameLine3D(ReferenceFrame referenceFrame, LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      setIncludingFrame(referenceFrame, lineSegment2DReadOnly);
   }

   /**
    * Creates a new line and initializes it to go through the endpoints of the given line segment.
    * <p>
    * The reference frame is initialized to {@code ReferenceFrame.getWorldFrame()}.
    * </p>
    *
    * @param lineSegment3DReadOnly the line segment to copy. Not modified.
    */
   public FrameLine3D(LineSegment3DReadOnly lineSegment3DReadOnly)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), lineSegment3DReadOnly);
   }

   /**
    * Creates a new line and initializes it to go through the endpoints of the given line segment.
    * <p>
    * The reference frame is initialized to {@code ReferenceFrame.getWorldFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the given line segment is expressed. The
    *           initial frame for this frame line.
    * @param lineSegment3DReadOnly the line segment to copy. Not modified.
    */
   public FrameLine3D(ReferenceFrame referenceFrame, LineSegment3DReadOnly lineSegment3DReadOnly)
   {
      setIncludingFrame(referenceFrame, lineSegment3DReadOnly);
   }

   /**
    * Creates a new line and initializes it to other.
    *
    * @param frameLine2DReadOnly the other line to copy. Not modified.
    */
   public FrameLine3D(FrameLine2DReadOnly frameLine2DReadOnly)
   {
      setIncludingFrame(frameLine2DReadOnly);
   }

   /**
    * Creates a new line and initializes it to other.
    *
    * @param other the other line to copy. Not modified.
    */
   public FrameLine3D(FrameLine3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /**
    * Creates a new line and initializes it to go through the endpoints of the given line segment.
    *
    * @param frameLineSegment2DReadOnly the line segment used to initialize this frame line. Not modified.
    */
   public FrameLine3D(FrameLineSegment2DReadOnly frameLineSegment2DReadOnly)
   {
      setIncludingFrame(frameLineSegment2DReadOnly);
   }

   /**
    * Creates a new line and initializes it to go through the endpoints of the given line segment.
    *
    * @param frameLineSegment3DReadOnly the line segment used to initialize this frame line. Not modified.
    */
   public FrameLine3D(FrameLineSegment3DReadOnly frameLineSegment3DReadOnly)
   {
      setIncludingFrame(frameLineSegment3DReadOnly);
   }

   /**
    * Creates a new line and initializes it to go through the given points.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the reference
    *            frame.
    */
   public FrameLine3D(FramePoint3DReadOnly firstPointOnLine, FramePoint3DReadOnly secondPointOnLine)
   {
      setIncludingFrame(firstPointOnLine, secondPointOnLine);
   }

   /**
    * Creates a new line and initializes it to the given point and direction.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the reference
    *            frame.
    */
   public FrameLine3D(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection)
   {
      setIncludingFrame(pointOnLine, lineDirection);
   }

   /** {@inheritDoc} */
   @Override
   public void set(FrameLine3D other)
   {
      FrameLine3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint3DBasics getPoint()
   {
      return point;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getDirection()
   {
      return direction;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /**
    * Tests on a per-component basis on the point and vector if this line is equal to {@code other}
    * with the tolerance {@code epsilon}. This method will return {@code false} if the two lines are
    * physically the same but either the point or vector of each line is different. For instance, if
    * {@code this.point == other.point} and {@code this.direction == - other.direction}, the two lines
    * are physically the same but this method returns {@code false}.
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two lines are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameLine3D other, double epsilon)
   {
      return FrameLine3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two lines are geometrically similar.
    * <p>
    * Two lines are considered geometrically equal is they are collinear, pointing toward the same or
    * opposite direction.
    * </p>
    *
    * @param other the line to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two lines represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   @Override
   public boolean geometricallyEquals(FrameLine3D other, double epsilon)
   {
      return FrameLine3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameLine3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((FrameLine3DReadOnly) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Provides a {@code String} representation of this frame line 3D as follows:<br>
    * Line 3D: point = (x, y, z), direction = (x, y, z)-worldFrame
    *
    * @return the {@code String} representing this line 3D.
    */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getLine3DString(this) + "-" + referenceFrame;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this line 3D.
    *
    * @return the hash code value for this line 3D.
    */
   @Override
   public int hashCode()
   {
      return line.hashCode();
   }
}
