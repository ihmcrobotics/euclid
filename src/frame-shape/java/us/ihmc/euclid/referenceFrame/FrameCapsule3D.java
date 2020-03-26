package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameCapsule3D implements FrameCapsule3DBasics, GeometryObject<FrameCapsule3D>
{
   private ReferenceFrame referenceFrame;
   /** Position of this capsule's center. */
   private final FixedFramePoint3DBasics position = EuclidFrameFactories.newFixedFramePoint3DBasics(this);
   /** Axis of revolution of this capsule. */
   private final FixedFrameVector3DBasics axis = EuclidFrameFactories.newFixedFrameVector3DBasics(this);
   /** This capsule radius. */
   private double radius;
   /** This capsule length. */
   private double length;
   /** This capsule half-length. */
   private double halfLength;

   /** Position of the top half-sphere center linked to this capsule properties. */
   private final FramePoint3DReadOnly topCenter = EuclidFrameFactories.newLinkedFramePoint3DReadOnly(() -> halfLength * axis.getX() + position.getX(),
                                                                                                     () -> halfLength * axis.getY() + position.getY(),
                                                                                                     () -> halfLength * axis.getZ() + position.getZ(),
                                                                                                     this);
   /** Position of the bottom half-sphere center linked to this capsule properties. */
   private final FramePoint3DReadOnly bottomCenter = EuclidFrameFactories.newLinkedFramePoint3DReadOnly(() -> -halfLength * axis.getX() + position.getX(),
                                                                                                        () -> -halfLength * axis.getY() + position.getY(),
                                                                                                        () -> -halfLength * axis.getZ() + position.getZ(),
                                                                                                        this);

   public FrameCapsule3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameCapsule3D(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 1.0, 0.5);
   }

   public FrameCapsule3D(ReferenceFrame referenceFrame, double length, double radius)
   {
      setReferenceFrame(referenceFrame);
      setSize(length, radius);
   }

   public FrameCapsule3D(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      setIncludingFrame(referenceFrame, position, axis, length, radius);
   }

   public FrameCapsule3D(FramePoint3DReadOnly position, FrameVector3DReadOnly axis, double length, double radius)
   {
      setIncludingFrame(position, axis, length, radius);
   }

   public FrameCapsule3D(ReferenceFrame referenceFrame, Capsule3DReadOnly other)
   {
      setIncludingFrame(referenceFrame, other);
   }

   public FrameCapsule3D(FrameCapsule3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   @Override
   public void set(FrameCapsule3D other)
   {
      FrameCapsule3DBasics.super.set(other);
   }

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
   public FixedFrameVector3DBasics getAxis()
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

   @Override
   public FrameCapsule3D copy()
   {
      return new FrameCapsule3D(this);
   }

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    *
    * @param other   the other capsule to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two capsules are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameCapsule3D other, double epsilon)
   {
      return FrameCapsule3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two capsules are geometrically
    * similar.
    *
    * @param other   the capsule to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the capsules represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(FrameCapsule3D other, double epsilon)
   {
      return FrameCapsule3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Capsule3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameCapsule3DReadOnly)
         return FrameCapsule3DBasics.super.equals((FrameCapsule3DReadOnly) object);
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
    * Provides a {@code String} representation of this capsule 3D as follows:<br>
    * Capsule 3D: [position: (-0.362, -0.617, 0.066 ), axis: ( 0.634, -0.551, -0.543 ), length: 0.170,
    * radius: 0.906]
    *
    * @return the {@code String} representing this capsule 3D.
    */
   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getFrameCapsule3DString(this);
   }
}
