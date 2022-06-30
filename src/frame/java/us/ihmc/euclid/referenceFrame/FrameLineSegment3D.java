package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DBasics;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

/**
 * {@code FrameLineSegment3D} is a 3D line segment expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link LineSegment3DBasics}, a {@link ReferenceFrame} is associated
 * to a {@code FrameLineSegment3D}. This allows, for instance, to enforce, at runtime, that
 * operations on line segments occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a point in different
 * reference frames.
 * </p>
 * <p>
 * Because a {@code FrameLineSegment3D} extends {@code LineSegment3DBasics}, it is compatible with
 * methods only requiring {@code LineSegment3DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameLineSegment3D}.
 * </p>
 */
public class FrameLineSegment3D implements FrameLineSegment3DBasics, Settable<FrameLineSegment3D>
{
   /** The reference frame in which this line is expressed. */
   private ReferenceFrame referenceFrame;
   private final FixedFramePoint3DBasics firstEndpoint = EuclidFrameFactories.newFixedFramePoint3DBasics(this);
   private final FixedFramePoint3DBasics secondEndpoint = EuclidFrameFactories.newFixedFramePoint3DBasics(this);

   /**
    * Default constructor that initializes both endpoints of this line segment to zero and its
    * reference frame to {@code ReferenceFrame.getWorldFrame()}.
    */
   public FrameLineSegment3D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Default constructor that initializes both endpoints of this line segment to zero and its
    * reference frame to {@code referenceFrame}.
    *
    * @param referenceFrame the initial reference frame for this line segment.
    */
   public FrameLineSegment3D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new line segment and initializes it to be same as the given line segment.
    * <p>
    * The reference frame is initialized to {@code ReferenceFrame.getWorldFrame()}.
    * </p>
    *
    * @param lineSegment3DReadOnly the other line segment to copy. Not modified.
    */
   public FrameLineSegment3D(LineSegment3DReadOnly lineSegment3DReadOnly)
   {
      this(ReferenceFrame.getWorldFrame(), lineSegment3DReadOnly);
   }

   /**
    * Creates a new line segment and initializes it to be same as the given line segment.
    *
    * @param referenceFrame        the initial reference frame for this line segment.
    * @param lineSegment3DReadOnly the other line segment to copy. Not modified.
    */
   public FrameLineSegment3D(ReferenceFrame referenceFrame, LineSegment3DReadOnly lineSegment3DReadOnly)
   {
      setIncludingFrame(referenceFrame, lineSegment3DReadOnly);
   }

   /**
    * Creates a new line segment and initializes it to be same as the given line segment.
    *
    * @param other the other line segment to copy. Not modified.
    */
   public FrameLineSegment3D(FrameLineSegment3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void set(FrameLineSegment3D other)
   {
      FrameLineSegment3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint3DBasics getFirstEndpoint()
   {
      return firstEndpoint;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint3DBasics getSecondEndpoint()
   {
      return secondEndpoint;
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
      if (object instanceof FrameLineSegment3DReadOnly)
         return equals((EuclidFrameGeometry) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this frame line segment 3D as follows:<br>
    * Line segment 3D: 1st endpoint = ( 0.174, 0.732, -0.222 ), 2nd endpoint = (-0.558, -0.380, 0.130
    * )-worldFrame
    *
    * @return the {@code String} representing this line segment 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this line segment
    * 3D.
    *
    * @return the hash code value for this line 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(firstEndpoint, secondEndpoint);
   }
}
