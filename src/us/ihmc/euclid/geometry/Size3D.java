package us.ihmc.euclid.geometry;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
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
    */
   public Size3D(double length, double width, double height)
   {
      this.length = length;
      this.width = width;
      this.height = height;
   }

   /**
    * Alias for X
    */
   public double getLength()
   {
      return getX();
   }

   /**
    * Alias for Y
    */
   public double getWidth()
   {
      return getY();
   }

   /**
    * Alias for Z
    */
   public double getHeight()
   {
      return getZ();
   }

   /**
    * Alias for X
    */
   public void setLength(double length)
   {
      setX(length);
   }

   /**
    * Alias for Y
    */
   public void setWidth(double width)
   {
      setY(width);
   }

   /**
    * Alias for Z
    */
   public void setHeight(double height)
   {
      setZ(height);
   }

   /**
    * Alias for X, Y, Z
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
}