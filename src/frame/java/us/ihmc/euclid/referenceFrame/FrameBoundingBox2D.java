package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * A {@link FrameBoundingBox2D} can be used to define from a set of minimum and maximum coordinates
 * an axis-aligned bounding box in the XY-plane that is expressed in a given reference frame..
 */
public class FrameBoundingBox2D implements FrameBoundingBox2DBasics
{
   /** The reference frame is which this bounding box is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** The minimum coordinates of this bounding box. */
   private final FixedFramePoint2DBasics minPoint = EuclidFrameFactories.newFixedFramePoint2DBasics(this);
   /** The maximum coordinates of this bounding box. */
   private final FixedFramePoint2DBasics maxPoint = EuclidFrameFactories.newFixedFramePoint2DBasics(this);

   /**
    * Creates a new bounding box expressed in {@link ReferenceFrame#getWorldFrame()} initialized with
    * both its minimum and maximum coordinates to ({@code Double.NaN}, {@code Double.NaN}).
    */
   public FrameBoundingBox2D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new bounding box expressed in {@code referenceFrame} initialized with both its minimum
    * and maximum coordinates to ({@code Double.NaN}, {@code Double.NaN}).
    *
    * @param referenceFrame the initial frame for this frame bounding box.
    */
   public FrameBoundingBox2D(ReferenceFrame referenceFrame)
   {
      setToNaN(referenceFrame);
   }

   /**
    * Creates a new bounding box and initializes it to the given minimum and maximum coordinates and
    * sets its reference frame.
    *
    * @param referenceFrame the initial frame for this frame bounding box.
    * @param min            the minimum coordinates for this. Not modified.
    * @param max            the maximum coordinates for this. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *                          coordinate on the same axis.
    */
   public FrameBoundingBox2D(ReferenceFrame referenceFrame, Point2DReadOnly min, Point2DReadOnly max)
   {
      setIncludingFrame(referenceFrame, min, max);
   }

   /**
    * Creates a new bounding box and initializes it to the given minimum and maximum coordinates and
    * sets its reference frame.
    *
    * @param min the minimum coordinates for this. Not modified.
    * @param max the maximum coordinates for this. Not modified.
    * @throws RuntimeException                if any of the minimum coordinates is strictly greater
    *                                         than the maximum coordinate on the same axis.
    * @throws ReferenceFrameMismatchException if {@code min} and {@code max} are not expressed in the
    *                                         same reference frame
    */
   public FrameBoundingBox2D(FramePoint2DReadOnly min, FramePoint2DReadOnly max)
   {
      setIncludingFrame(min, max);
   }

   /**
    * Creates a new bounding box and initializes it to {@code boundingBox2DReadOnly} and sets its
    * reference frame.
    *
    * @param referenceFrame        the initial frame for this frame bounding box.
    * @param boundingBox2DReadOnly the other bounding box used to initialize this. Not modified.
    */
   public FrameBoundingBox2D(ReferenceFrame referenceFrame, BoundingBox2DReadOnly boundingBox2DReadOnly)
   {
      setIncludingFrame(referenceFrame, boundingBox2DReadOnly);
   }

   /**
    * Creates a new bounding box and initializes it to {@code other} and sets its reference frame.
    *
    * @param other the other bounding box used to initialize this. Not modified.
    */
   public FrameBoundingBox2D(FrameBoundingBox2DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint2DBasics getMinPoint()
   {
      return minPoint;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint2DBasics getMaxPoint()
   {
      return maxPoint;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameBoundingBox2DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two bounding boxes have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameBoundingBox2DReadOnly)
         return FrameBoundingBox2DBasics.super.equals((FrameBoundingBox2DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the min and max coordinates of this bounding box.
    *
    * @return the hash code value for this bounding box.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(minPoint, maxPoint);
   }

   /**
    * Provides a {@code String} representation of this bounding box 2D as follows:
    *
    * <pre>
    * Bounding Box 2D: min = ( 0.174,  0.732 ), max = (-0.558, -0.380 ), worldFrame
    * </pre>
    *
    * @return the {@code String} representing this bounding box 2D.
    */
   @Override
   public String toString()
   {
      return FrameBoundingBox2DBasics.super.toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
