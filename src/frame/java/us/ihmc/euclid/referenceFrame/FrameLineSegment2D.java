package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * {@code FrameLineSegment2D} is a 2D line segment expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link LineSegment2DBasics}, a {@link ReferenceFrame} is associated
 * to a {@code FrameLineSegment2D}. This allows, for instance, to enforce, at runtime, that
 * operations on line segments occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a point in different
 * reference frames.
 * </p>
 * <p>
 * Because a {@code FrameLineSegment2D} extends {@code LineSegment2DBasics}, it is compatible with
 * methods only requiring {@code LineSegment2DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameLineSegment2D}.
 * </p>
 */
public class FrameLineSegment2D implements FrameLineSegment2DBasics, Settable<FrameLineSegment2D>
{
   /** The reference frame in which this line is expressed. */
   private ReferenceFrame referenceFrame;
   private final FixedFramePoint2DBasics firstEndpoint = EuclidFrameFactories.newFixedFramePoint2DBasics(this);
   private final FixedFramePoint2DBasics secondEndpoint = EuclidFrameFactories.newFixedFramePoint2DBasics(this);
   /** Rigid-body transform used to perform garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   /**
    * Default constructor that initializes both endpoints of this line segment to zero and its
    * reference frame to {@code ReferenceFrame.getWorldFrame()}.
    */
   public FrameLineSegment2D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Default constructor that initializes both endpoints of this line segment to zero and its
    * reference frame to {@code referenceFrame}.
    *
    * @param referenceFrame the initial reference frame for this line segment.
    */
   public FrameLineSegment2D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new line segment and initializes it to be same as the given line segment.
    * <p>
    * The reference frame is initialized to {@code ReferenceFrame.getWorldFrame()}.
    * </p>
    *
    * @param lineSegment2DReadOnly the other line segment to copy. Not modified.
    */
   public FrameLineSegment2D(LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), lineSegment2DReadOnly);
   }

   /**
    * Creates a new line segment and initializes it to be same as the given line segment.
    *
    * @param referenceFrame        the initial reference frame for this line segment.
    * @param lineSegment2DReadOnly the other line segment to copy. Not modified.
    */
   public FrameLineSegment2D(ReferenceFrame referenceFrame, LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      setIncludingFrame(referenceFrame, lineSegment2DReadOnly);
   }

   /**
    * Creates a new line segment and initializes it to be same as the given line segment.
    *
    * @param other the other line segment to copy. Not modified.
    */
   public FrameLineSegment2D(FrameLineSegment2DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /**
    * Creates a new line segment and initializes to with the given endpoints.
    *
    * @param firstEndpoint  new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   public FrameLineSegment2D(FramePoint2DReadOnly firstEndpoint, FramePoint2DReadOnly secondEndpoint)
   {
      setIncludingFrame(firstEndpoint, secondEndpoint);
   }

   /** {@inheritDoc} */
   @Override
   public void set(FrameLineSegment2D other)
   {
      FrameLineSegment2DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint2DBasics getFirstEndpoint()
   {
      return firstEndpoint;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint2DBasics getSecondEndpoint()
   {
      return secondEndpoint;
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
    * <p>
    * If the two line segments have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if the two line segments are exactly equal component-wise and are expressed
    *         in the same reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameLineSegment2DReadOnly)
         return equals((EuclidFrameGeometry) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this frame line segment 2D as follows:<br>
    * Line segment 2D: 1st endpoint = ( 0.174, 0.732, -0.222 ), 2nd endpoint = (-0.558, -0.380, 0.130
    * )-worldFrame
    *
    * @return the {@code String} representing this line segment 2D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this line segment
    * 2D.
    *
    * @return the hash code value for this line segment 2D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(firstEndpoint, secondEndpoint);
   }
}