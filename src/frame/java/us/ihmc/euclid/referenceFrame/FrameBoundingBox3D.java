package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * A {@link BoundingBox3D} can be used to defines from a set of minimum and maximum coordinates an
 * axis-aligned bounding box that is expressed in a reference frame.
 */
public class FrameBoundingBox3D implements FrameBoundingBox3DBasics, Settable<FrameBoundingBox3D>

{
   /** The reference frame is which this bounding box is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** The minimum coordinates of this bounding box. */
   private final FixedFramePoint3DBasics minPoint = EuclidFrameFactories.newFixedFramePoint3DBasics(this);
   /** The maximum coordinates of this bounding box. */
   private final FixedFramePoint3DBasics maxPoint = EuclidFrameFactories.newFixedFramePoint3DBasics(this);

   /**
    * Creates a new bounding box expressed in {@link ReferenceFrame#getWorldFrame()} initialized with
    * both its minimum and maximum coordinates to ({@code Double.NaN}, {@code Double.NaN}).
    */
   public FrameBoundingBox3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new bounding box expressed in {@code referenceFrame} initialized with both its minimum
    * and maximum coordinates to ({@code Double.NaN}, {@code Double.NaN}, {@code Double.NaN}).
    *
    * @param referenceFrame the initial frame for this frame bounding box.
    */
   public FrameBoundingBox3D(ReferenceFrame referenceFrame)
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
   public FrameBoundingBox3D(ReferenceFrame referenceFrame, Point3DReadOnly min, Point3DReadOnly max)
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
   public FrameBoundingBox3D(FramePoint3DReadOnly min, FramePoint3DReadOnly max)
   {
      setIncludingFrame(min, max);
   }

   /**
    * Creates a new bounding box and initializes it to {@code boundingBox3DReadOnly} and sets its
    * reference frame.
    *
    * @param referenceFrame        the initial frame for this frame bounding box.
    * @param boundingBox3DReadOnly the other bounding box used to initialize this. Not modified.
    */
   public FrameBoundingBox3D(ReferenceFrame referenceFrame, BoundingBox3DReadOnly boundingBox3DReadOnly)
   {
      setIncludingFrame(referenceFrame, boundingBox3DReadOnly);
   }

   /**
    * Creates a new bounding box and initializes it to {@code other} and sets its reference frame.
    *
    * @param other the other bounding box used to initialize this. Not modified.
    */
   public FrameBoundingBox3D(FrameBoundingBox3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void set(FrameBoundingBox3D other)
   {
      FrameBoundingBox3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint3DBasics getMinPoint()
   {
      return minPoint;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint3DBasics getMaxPoint()
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
    * {@link #equals(FrameBoundingBox3DReadOnly)}, it returns {@code false} otherwise.
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
      if (object instanceof FrameBoundingBox3DReadOnly)
         return FrameBoundingBox3DBasics.super.equals((FrameBoundingBox3DReadOnly) object);
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
    * Provides a {@code String} representation of this bounding box 3D as follows:
    *
    * <pre>
    * Bounding Box 3D: min = ( 0.174,  0.732, -0.222 ), max = (-0.558, -0.380,  0.130 ), worldFrame
    * </pre>
    *
    * @return the {@code String} representing this bounding box 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
