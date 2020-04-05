package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameSphere3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameSphere3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * {@code Sphere3D} represents a 3D sphere expressed in a given reference frame that is defined by
 * its radius and with its origin at its center.
 */
public class FrameSphere3D implements FrameSphere3DBasics, GeometryObject<FrameSphere3D>
{
   /** The reference frame in which this shape is expressed. */
   private ReferenceFrame referenceFrame;
   /** The position of the center of this sphere. */
   private final FixedFramePoint3DBasics position = EuclidFrameFactories.newFixedFramePoint3DBasics(this);
   /** The radius of this sphere. */
   private double radius;

   /**
    * Creates a new sphere 3D with a radius of {@code 1} and initializes its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameSphere3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new sphere 3D with a radius of {@code 1} and initializes its reference frame.
    *
    * @param referenceFrame this shape initial reference frame.
    */
   public FrameSphere3D(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 1.0);
   }

   /**
    * Creates a new sphere 3D and initializes its radius.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param radius         the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public FrameSphere3D(ReferenceFrame referenceFrame, double radius)
   {
      setReferenceFrame(referenceFrame);
      setRadius(radius);
   }

   /**
    * Creates a new sphere 3D and initializes its position and radius.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param center         the coordinates of this sphere. Not modified.
    * @param radius         the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public FrameSphere3D(ReferenceFrame referenceFrame, Point3DReadOnly center, double radius)
   {
      setIncludingFrame(referenceFrame, center, radius);
   }

   /**
    * Creates a new sphere 3D and initializes its position and radius.
    *
    * @param center the coordinates of this sphere. Not modified.
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public FrameSphere3D(FramePoint3DReadOnly center, double radius)
   {
      setIncludingFrame(center, radius);
   }

   /**
    * Creates a new sphere 3D and initializes its position and radius.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param centerX        the x-coordinate of this sphere. Not modified.
    * @param centerY        the y-coordinate of this sphere. Not modified.
    * @param centerZ        the z-coordinate of this sphere. Not modified.
    * @param radius         the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public FrameSphere3D(ReferenceFrame referenceFrame, double centerX, double centerY, double centerZ, double radius)
   {
      setIncludingFrame(referenceFrame, centerX, centerY, centerZ, radius);
   }

   /**
    * Creates a new sphere 3D identical to {@code other}
    *
    * @param referenceFrame this shape initial reference frame.
    * @param other          the other sphere to copy. Not modified.
    */
   public FrameSphere3D(ReferenceFrame referenceFrame, Sphere3DReadOnly other)
   {
      setIncludingFrame(referenceFrame, other);
   }

   /**
    * Creates a new sphere 3D identical to {@code other}
    *
    * @param other the other sphere to copy. Not modified.
    */
   public FrameSphere3D(FrameSphere3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint3DBasics getPosition()
   {
      return position;
   }

   /**
    * Gets the radius of this sphere.
    *
    * @return the value of the radius.
    */
   @Override
   public double getRadius()
   {
      return radius;
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /**
    * Sets the radius of this sphere.
    *
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   @Override
   public void setRadius(double radius)
   {
      if (radius < 0.0)
         throw new IllegalArgumentException("The radius of a Sphere 3D cannot be negative.");
      this.radius = radius;
   }

   /** {@inheritDoc} */
   @Override
   public FrameSphere3D copy()
   {
      return new FrameSphere3D(this);
   }

   /** {@inheritDoc} */
   @Override
   public void set(FrameSphere3D other)
   {
      FrameSphere3DBasics.super.set(other);
   }

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    * <p>
    * If the two ellipsoids have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other   the other sphere to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two spheres are equal component-wise and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameSphere3D other, double epsilon)
   {
      return FrameSphere3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two spheres are geometrically similar,
    * i.e. the position of each sphere is geometrically similar given {@code epsilon} and the
    * difference between the radius of each sphere is less than or equal to {@code epsilon}.
    *
    * @param other   the sphere to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two spheres represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   @Override
   public boolean geometricallyEquals(FrameSphere3D other, double epsilon)
   {
      return FrameSphere3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameSphere3DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two ellipsoids have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameSphere3DReadOnly)
         return FrameSphere3DBasics.super.equals((FrameSphere3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this sphere 3D.
    *
    * @return the hash code value for this sphere 3D.
    */
   @Override
   public int hashCode()
   {
      long hash = EuclidHashCodeTools.addToHashCode(position.hashCode(), radius);
      return EuclidHashCodeTools.toIntHashCode(hash);
   }

   /**
    * Provides a {@code String} representation of this sphere 3D as follows:
    *
    * <pre>
    * Sphere 3D: [position: (-0.362, -0.617,  0.066 ), radius:  0.906] - worldFrame
    * </pre>
    *
    * @return the {@code String} representing this sphere 3D.
    */
   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getFrameSphere3DString(this);
   }
}
