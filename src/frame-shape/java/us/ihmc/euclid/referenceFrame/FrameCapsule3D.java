package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameUnitVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Implementation of a capsule 3D expressed in a given reference frame.
 * <p>
 * A capsule 3D is represented by its length, i.e. the distance separating the center of the two
 * half-spheres, its radius, the position of its center, and its axis of revolution.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class FrameCapsule3D implements FrameCapsule3DBasics, Settable<FrameCapsule3D>
{
   /** The reference frame in which this shape is expressed. */
   private ReferenceFrame referenceFrame;
   /** Position of this capsule's center. */
   private final FixedFramePoint3DBasics position = EuclidFrameFactories.newFixedFramePoint3DBasics(this);
   /** Axis of revolution of this capsule. */
   private final FixedFrameUnitVector3DBasics axis = EuclidFrameFactories.newFixedFrameUnitVector3DBasics(this, Axis3D.Z);
   /** This capsule radius. */
   private double radius;
   /** This capsule length. */
   private double length;
   /** This capsule half-length. */
   private double halfLength;

   /** Position of the top half-sphere center linked to this capsule properties. */
   private final FramePoint3DReadOnly topCenter = EuclidFrameFactories.newLinkedFramePoint3DReadOnly(this,
                                                                                                     () -> halfLength * axis.getX() + position.getX(),
                                                                                                     () -> halfLength * axis.getY() + position.getY(),
                                                                                                     () -> halfLength * axis.getZ() + position.getZ());
   /** Position of the bottom half-sphere center linked to this capsule properties. */
   private final FramePoint3DReadOnly bottomCenter = EuclidFrameFactories.newLinkedFramePoint3DReadOnly(this,
                                                                                                        () -> -halfLength * axis.getX() + position.getX(),
                                                                                                        () -> -halfLength * axis.getY() + position.getY(),
                                                                                                        () -> -halfLength * axis.getZ() + position.getZ());

   /**
    * Creates a new capsule which axis is along the z-axis, a length of 1, and radius of 0.5 and
    * initializes its reference frame to {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameCapsule3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new capsule which axis is along the z-axis, a length of 1, and radius of 0.5 and
    * initializes its reference frame.
    *
    * @param referenceFrame this shape initial reference frame.
    */
   public FrameCapsule3D(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 1.0, 0.5);
   }

   /**
    * Creates a new capsule which axis is along the z-axis and initializes its size.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param length         the length of this capsule.
    * @param radius         the radius of this capsule.
    * @throws IllegalArgumentException if {@code length} or {@code radius} is negative.
    */
   public FrameCapsule3D(ReferenceFrame referenceFrame, double length, double radius)
   {
      setReferenceFrame(referenceFrame);
      setSize(length, radius);
   }

   /**
    * Creates a new capsule 3D and initializes its pose and size.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param position       the position of the center. Not modified.
    * @param axis           the axis of revolution. Not modified.
    * @param length         the length of this capsule.
    * @param radius         the radius of this capsule.
    * @throws IllegalArgumentException if {@code length} or {@code radius} is negative.
    */
   public FrameCapsule3D(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      setIncludingFrame(referenceFrame, position, axis, length, radius);
   }

   /**
    * Creates a new capsule 3D and initializes its pose and size.
    *
    * @param position the position of the center. Not modified.
    * @param axis     the axis of revolution. Not modified.
    * @param length   the length of this capsule.
    * @param radius   the radius of this capsule.
    * @throws IllegalArgumentException        if {@code length} or {@code radius} is negative.
    * @throws ReferenceFrameMismatchException if the frame argument are not expressed in the same
    *                                         reference frame.
    */
   public FrameCapsule3D(FramePoint3DReadOnly position, FrameVector3DReadOnly axis, double length, double radius)
   {
      setIncludingFrame(position, axis, length, radius);
   }

   /**
    * Creates a new capsule 3D identical to {@code other}.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param other          the other capsule to copy. Not modified.
    */
   public FrameCapsule3D(ReferenceFrame referenceFrame, Capsule3DReadOnly other)
   {
      setIncludingFrame(referenceFrame, other);
   }

   /**
    * Creates a new capsule 3D identical to {@code other}.
    *
    * @param other the other capsule to copy. Not modified.
    */
   public FrameCapsule3D(FrameCapsule3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void set(FrameCapsule3D other)
   {
      FrameCapsule3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setRadius(double radius)
   {
      if (radius < 0.0)
         throw new IllegalArgumentException("The radius of a Capsule3D cannot be negative: " + radius);
      this.radius = radius;
   }

   /** {@inheritDoc} */
   @Override
   public void setLength(double length)
   {
      if (length < 0.0)
         throw new IllegalArgumentException("The length of a Capsule3D cannot be negative: " + length);
      this.length = length;
      halfLength = 0.5 * length;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public double getRadius()
   {
      return radius;
   }

   /** {@inheritDoc} */
   @Override
   public double getLength()
   {
      return length;
   }

   /** {@inheritDoc} */
   @Override
   public double getHalfLength()
   {
      return halfLength;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint3DBasics getPosition()
   {
      return position;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameUnitVector3DBasics getAxis()
   {
      return axis;
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getTopCenter()
   {
      return topCenter;
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getBottomCenter()
   {
      return bottomCenter;
   }

   /** {@inheritDoc} */
   @Override
   public FrameCapsule3D copy()
   {
      return new FrameCapsule3D(this);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(EuclidFrameGeometry)}, it returns {@code false} otherwise.
    * <p>
    * If the two capsules have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameCapsule3DReadOnly)
         return equals((EuclidFrameGeometry) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this capsule 3D.
    *
    * @return the hash code value for this capsule 3D.
    */
   @Override
   public int hashCode()
   {
      long hash = 1L;
      hash = EuclidHashCodeTools.toLongHashCode(length, radius);
      hash = EuclidHashCodeTools.combineHashCode(hash, EuclidHashCodeTools.toLongHashCode(position, axis));
      return EuclidHashCodeTools.toIntHashCode(hash);
   }

   /**
    * Provides a {@code String} representation of this capsule 3D as follows:
    *
    * <pre>
    * Capsule 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906] - worldFrame
    * </pre>
    *
    * @return the {@code String} representing this capsule 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
