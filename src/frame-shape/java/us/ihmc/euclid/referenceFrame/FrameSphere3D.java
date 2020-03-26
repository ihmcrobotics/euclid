package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameSphere3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameSphere3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class FrameSphere3D implements FrameSphere3DBasics, GeometryObject<FrameSphere3D>
{
   private ReferenceFrame referenceFrame;
   private final FixedFramePoint3DBasics position = EuclidFrameFactories.newFixedFramePoint3DBasics(this);

   /** The radius of this sphere. */
   private double radius;

   public FrameSphere3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameSphere3D(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 1.0);
   }

   public FrameSphere3D(ReferenceFrame referenceFrame, double radius)
   {
      setReferenceFrame(referenceFrame);
      setRadius(radius);
   }

   public FrameSphere3D(ReferenceFrame referenceFrame, Point3DReadOnly center, double radius)
   {
      setIncludingFrame(referenceFrame, center, radius);
   }

   public FrameSphere3D(FramePoint3DReadOnly center, double radius)
   {
      setIncludingFrame(center, radius);
   }

   public FrameSphere3D(ReferenceFrame referenceFrame, double centerX, double centerY, double centerZ, double radius)
   {
      setIncludingFrame(referenceFrame, centerX, centerY, centerZ, radius);
   }

   public FrameSphere3D(ReferenceFrame referenceFrame, Sphere3DReadOnly other)
   {
      setIncludingFrame(referenceFrame, other);
   }

   public FrameSphere3D(FrameSphere3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

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

   @Override
   public FrameSphere3D copy()
   {
      return new FrameSphere3D(this);
   }

   @Override
   public void set(FrameSphere3D other)
   {
      FrameSphere3DBasics.super.set(other);
   }

   @Override
   public boolean epsilonEquals(FrameSphere3D other, double epsilon)
   {
      return FrameSphere3DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(FrameSphere3D other, double epsilon)
   {
      return FrameSphere3DBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameSphere3DReadOnly)
         return FrameSphere3DBasics.super.equals((FrameSphere3DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      long hash = EuclidHashCodeTools.addToHashCode(position.hashCode(), radius);
      return EuclidHashCodeTools.toIntHashCode(hash);
   }

   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getFrameSphere3DString(this);
   }
}
