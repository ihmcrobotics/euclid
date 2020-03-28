package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class FramePointShape3D implements FramePointShape3DBasics, GeometryObject<FramePointShape3D>
{
   private ReferenceFrame referenceFrame;
   private double x, y, z;

   public FramePointShape3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FramePointShape3D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FramePointShape3D(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      setIncludingFrame(referenceFrame, x, y, z);
   }

   public FramePointShape3D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      setIncludingFrame(referenceFrame, tuple3DReadOnly);
   }

   public FramePointShape3D(FrameTuple3DReadOnly tuple3DReadOnly)
   {
      setIncludingFrame(tuple3DReadOnly);
   }

   @Override
   public void set(FramePointShape3D other)
   {
      FramePointShape3DBasics.super.set(other);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   /** {@inheritDoc} */
   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   /** {@inheritDoc} */
   @Override
   public void setZ(double z)
   {
      this.z = z;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public double getX()
   {
      return x;
   }

   /** {@inheritDoc} */
   @Override
   public double getY()
   {
      return y;
   }

   /** {@inheritDoc} */
   @Override
   public double getZ()
   {
      return z;
   }

   @Override
   public FramePointShape3D copy()
   {
      return new FramePointShape3D(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FramePointShape3DReadOnly)
         return FramePointShape3DBasics.super.equals((FramePointShape3DReadOnly) object);
      else
         return false;
   }

   @Override
   public boolean epsilonEquals(FramePointShape3D other, double epsilon)
   {
      return FramePointShape3DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(FramePointShape3D other, double epsilon)
   {
      return FramePointShape3DBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.toLongHashCode(x, y, z);
      bits = EuclidHashCodeTools.addToHashCode(bits, referenceFrame);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getFramePointShape3DString(this);
   }
}
