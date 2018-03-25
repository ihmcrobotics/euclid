package us.ihmc.euclid.geometry;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

/**
 * Extension of a tuple 3D where x, y, and z have aliases for length, width, and height,
 * respectively.
 */
class Size3D implements Tuple3DBasics
{
   private double length;
   private double width;
   private double height;

   public Size3D()
   {
      setToZero();
   }

   /**
    * Alias for X, Y, Z
    * 
    * @param length the new length value, i.e. along the x-axis.
    * @param width the new width value, i.e. along the y-axis.
    * @param height the new height value, i.e. along the z-axis.
    */
   public Size3D(double length, double width, double height)
   {
      this.length = length;
      this.width = width;
      this.height = height;
   }

   /**
    * Alias for X
    * 
    * @return the length along the x-axis.
    */
   public double getLength()
   {
      return getX();
   }

   /**
    * Alias for Y
    * 
    * @return the width along the y-axis.
    */
   public double getWidth()
   {
      return getY();
   }

   /**
    * Alias for Z
    * 
    * @return the height along the z-axis.
    */
   public double getHeight()
   {
      return getZ();
   }

   /**
    * Alias for X
    * 
    * @param length the new length value, i.e. along the x-axis.
    */
   public void setLength(double length)
   {
      setX(length);
   }

   /**
    * Alias for Y
    * 
    * @param width the new width value, i.e. along the y-axis.
    */
   public void setWidth(double width)
   {
      setY(width);
   }

   /**
    * Alias for Z
    * 
    * @param height the new height value, i.e. along the z-axis.
    */
   public void setHeight(double height)
   {
      setZ(height);
   }

   /**
    * Alias for X, Y, Z
    * 
    * @param length the new length value, i.e. along the x-axis.
    * @param width the new width value, i.e. along the y-axis.
    * @param height the new height value, i.e. along the z-axis.
    */
   public void setLengthWidthHeight(double length, double width, double height)
   {
      set(length, width, height);
   }

   @Override
   public void setX(double x)
   {
      length = x;
   }

   @Override
   public void setY(double y)
   {
      width = y;
   }

   @Override
   public void setZ(double z)
   {
      height = z;
   }

   @Override
   public double getX()
   {
      return length;
   }

   @Override
   public double getY()
   {
      return width;
   }

   @Override
   public double getZ()
   {
      return height;
   }

   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple3DString(this);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      throw new UnsupportedOperationException();
   }
}
