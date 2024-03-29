package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.geometry.interfaces.Line2DBasics;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameUnitVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

/**
 * {@code FrameLine2D} is a 2D line expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Line2DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameLine2D}. This allows, for instance, to enforce, at runtime, that operations on lines
 * occur in the same coordinate system. Also, via the method {@link #changeFrame(ReferenceFrame)},
 * one can easily calculates the value of a point in different reference frames.
 * </p>
 * <p>
 * Because a {@code FrameLine2D} extends {@code Line2DBasics}, it is compatible with methods only
 * requiring {@code Line2DBasics}. However, these methods do NOT assert that the operation occur in
 * the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameLine2D}.
 * </p>
 */
public class FrameLine2D implements FrameLine2DBasics, Settable<FrameLine2D>
{
   /** The reference frame in which this line is expressed. */
   private ReferenceFrame referenceFrame;
   private final FixedFramePoint2DBasics point = EuclidFrameFactories.newFixedFramePoint2DBasics(this);
   private final FixedFrameUnitVector2DBasics direction = EuclidFrameFactories.newFixedFrameUnitVector2DBasics(this, Axis2D.X);
   /** Rigid-body transform used to perform garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   /**
    * Default constructor that initializes its {@code point} to zero, its {@code direction} to
    * {@link Axis2D#X} and the reference frame to {@code ReferenceFrame.getWorldFrame()}.
    */
   public FrameLine2D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Default constructor that initializes its {@code point} to zero, its {@code direction} to (1.0,
    * 0.0), and the reference frame to the given {@code referenceFrame}.
    *
    * @param referenceFrame the initial reference frame for this line.
    */
   public FrameLine2D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new line, initializes its point and direction from the given line and its reference
    * frame to {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param line2DReadOnly the line used to initialize the point and direction of this. Not modified.
    */
   public FrameLine2D(Line2DReadOnly line2DReadOnly)
   {
      this(ReferenceFrame.getWorldFrame(), line2DReadOnly);
   }

   /**
    * Creates a new line, initializes its point and direction from the given line and its reference
    * frame to {@code referenceFrame}.
    *
    * @param referenceFrame the initial reference frame for this line.
    * @param line2DReadOnly the line used to initialize the point and direction of this. Not modified.
    */
   public FrameLine2D(ReferenceFrame referenceFrame, Line2DReadOnly line2DReadOnly)
   {
      setIncludingFrame(referenceFrame, line2DReadOnly);
   }

   /**
    * Creates a new line, initializes it to go through the two given points in the given frame.
    *
    * @param referenceFrame    the initial reference frame for this line.
    * @param firstPointOnLine  first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    */
   public FrameLine2D(ReferenceFrame referenceFrame, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
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
   public FrameLine2D(ReferenceFrame referenceFrame, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
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
   public FrameLine2D(LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), lineSegment2DReadOnly);
   }

   /**
    * Creates a new line and initializes it to go through the endpoints of the given line segment.
    *
    * @param referenceFrame        the reference frame in which the given line segment is expressed.
    *                              The initial frame for this frame line.
    * @param lineSegment2DReadOnly the line segment to copy. Not modified.
    */
   public FrameLine2D(ReferenceFrame referenceFrame, LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      setIncludingFrame(referenceFrame, lineSegment2DReadOnly);
   }

   /**
    * Creates a new line and initializes it to other.
    *
    * @param other the other line to copy. Not modified.
    */
   public FrameLine2D(FrameLine2DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /**
    * Creates a new line and initializes it to go through the endpoints of the given line segment.
    *
    * @param frameLineSegment2DReadOnly the line segment used to. Not modified.
    */
   public FrameLine2D(FrameLineSegment2DReadOnly frameLineSegment2DReadOnly)
   {
      setIncludingFrame(frameLineSegment2DReadOnly);
   }

   /**
    * Creates a new line and initializes it to go through the given points.
    *
    * @param firstPointOnLine  first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the reference
    *                                         frame.
    */
   public FrameLine2D(FramePoint2DReadOnly firstPointOnLine, FramePoint2DReadOnly secondPointOnLine)
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
   public FrameLine2D(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      setIncludingFrame(pointOnLine, lineDirection);
   }

   /** {@inheritDoc} */
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
   public FixedFrameUnitVector2DBasics getDirection()
   {
      return direction;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
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
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(EuclidFrameGeometry)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameLine2DReadOnly)
         return equals((EuclidFrameGeometry) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this frame line 2D as follows:<br>
    * Line 2D: point = (x, y), direction = (x, y)-worldFrame
    *
    * @return the {@code String} representing this line 2D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this line 2D.
    *
    * @return the hash code value for this line 2D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(point, direction);
   }
}
