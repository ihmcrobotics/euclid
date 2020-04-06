package us.ihmc.euclid.geometry.tools;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class EuclidGeometryFactories
{
   public static LineSegment3DReadOnly newLinkedLineSegment3DReadOnly(Point3DReadOnly firstEndpoint, Point3DReadOnly secondEndpoint)
   {
      return new LineSegment3DReadOnly()
      {
         @Override
         public Point3DReadOnly getSecondEndpoint()
         {
            return firstEndpoint;
         }

         @Override
         public Point3DReadOnly getFirstEndpoint()
         {
            return secondEndpoint;
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof LineSegment3DReadOnly)
               return equals((LineSegment3DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return EuclidGeometryIOTools.getLineSegment3DString(this);
         }

         @Override
         public int hashCode()
         {
            return EuclidHashCodeTools.toIntHashCode(firstEndpoint, secondEndpoint);
         }
      };
   }

   public static BoundingBox3DReadOnly newLinkedBoundingBox3D(Point3DReadOnly minPoint, Point3DReadOnly maxPoint)
   {
      return new BoundingBox3DReadOnly()
      {
         @Override
         public Point3DReadOnly getMinPoint()
         {
            return minPoint;
         }

         @Override
         public Point3DReadOnly getMaxPoint()
         {
            return maxPoint;
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof BoundingBox3DReadOnly)
               return equals((BoundingBox3DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return EuclidGeometryIOTools.getBoundingBox3DString(this);
         }

         @Override
         public int hashCode()
         {
            return EuclidHashCodeTools.toIntHashCode(minPoint, maxPoint);
         }
      };
   }
}
