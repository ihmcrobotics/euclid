package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameUnitVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
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
public class FrameLine3D implements FrameLine3DBasics, Settable<FrameLine3D>
{
   /** The reference frame in which this line is expressed. */
   private ReferenceFrame referenceFrame;
   private final FixedFramePoint3DBasics point = EuclidFrameFactories.newFixedFramePoint3DBasics(this);
   private final FixedFrameUnitVector3DBasics direction = EuclidFrameFactories.newFixedFrameUnitVector3DBasics(this, Axis3D.X);

   /**
    * Default constructor that initializes its {@code point} to zero, its {@code direction} to
    * {@link Axis3D#X} and the reference frame to {@code ReferenceFrame.getWorldFrame()}.
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
    * @param referenceFrame    the initial reference frame for this line.
    * @param firstPointOnLine  first point on this line. Not modified.
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
    * @param pointOnLine    new point on this line. Not modified.
    * @param lineDirection  new direction of this line. Not modified.
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
    * @param referenceFrame        the reference frame in which the given line segment is expressed.
    *                              The initial frame for this frame line.
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
    * @param referenceFrame        the reference frame in which the given line segment is expressed.
    *                              The initial frame for this frame line.
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
    * @param frameLineSegment2DReadOnly the line segment used to initialize this frame line. Not
    *                                   modified.
    */
   public FrameLine3D(FrameLineSegment2DReadOnly frameLineSegment2DReadOnly)
   {
      setIncludingFrame(frameLineSegment2DReadOnly);
   }

   /**
    * Creates a new line and initializes it to go through the endpoints of the given line segment.
    *
    * @param frameLineSegment3DReadOnly the line segment used to initialize this frame line. Not
    *                                   modified.
    */
   public FrameLine3D(FrameLineSegment3DReadOnly frameLineSegment3DReadOnly)
   {
      setIncludingFrame(frameLineSegment3DReadOnly);
   }

   /**
    * Creates a new line and initializes it to go through the given points.
    *
    * @param firstPointOnLine  first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the reference
    *                                         frame.
    */
   public FrameLine3D(FramePoint3DReadOnly firstPointOnLine, FramePoint3DReadOnly secondPointOnLine)
   {
      setIncludingFrame(firstPointOnLine, secondPointOnLine);
   }

   /**
    * Creates a new line and initializes it to the given point and direction.
    *
    * @param pointOnLine   new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the reference
    *                                         frame.
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
   public FixedFrameUnitVector3DBasics getDirection()
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
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(EuclidFrameGeometry)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameLine3DReadOnly)
         return equals((EuclidFrameGeometry) object);
      else
         return false;
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
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this line 3D.
    *
    * @return the hash code value for this line 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(point, direction);
   }
}
