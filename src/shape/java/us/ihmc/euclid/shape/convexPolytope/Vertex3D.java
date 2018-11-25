package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * This class stores the location of a point which is the vertex of a polytope A list of polytope
 * edges originating from this vertex is also stored for ease of algorithm design Faces to which
 * this vertex belongs can be accessed by iterating through the list of edges
 *
 * @author Apoorv S
 *
 */
public class Vertex3D extends Vertex3DBasics
{
   private double x, y, z;

   public Vertex3D()
   {
      setToZero();
   }

   public Vertex3D(double x, double y, double z)
   {
      set(x, y, z);
   }

   public Vertex3D(Point3DReadOnly position)
   {
      set(position);
   }

   public Vertex3D(Vertex3DBasics vertex)
   {
      set(vertex);
   }

   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   @Override
   public void setZ(double z)
   {
      this.z = z;
   }

   @Override
   public double getX()
   {
      return x;
   }

   @Override
   public double getY()
   {
      return y;
   }

   @Override
   public double getZ()
   {
      return z;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Vertex3DReadOnly)
         return equals((Vertex3DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      long hashCode = 1L;
      hashCode = EuclidHashCodeTools.addToHashCode(hashCode, x);
      hashCode = EuclidHashCodeTools.addToHashCode(hashCode, y);
      hashCode = EuclidHashCodeTools.addToHashCode(hashCode, z);
      return EuclidHashCodeTools.toIntHashCode(hashCode);
   }
}
