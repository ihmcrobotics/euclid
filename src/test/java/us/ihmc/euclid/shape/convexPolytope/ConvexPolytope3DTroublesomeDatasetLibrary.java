package us.ihmc.euclid.shape.convexPolytope;

import java.util.*;

import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class ConvexPolytope3DTroublesomeDatasetLibrary
{
   public static class DatasetGJKNullPointerExceptionBug1Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug1Original()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.1727226445760459, 0.3408246844828842, 0.0707692371805856));
         pointsBeforeIssue.add(new Point3D(-0.0010094559634557, 0.1846852819495834, -0.0004136019841320));
         pointsBeforeIssue.add(new Point3D(0.0131019603409140, 0.0679884843470001, 0.0053682349594210));
         pointsBeforeIssue.add(new Point3D(0.0268762363673278, 0.0420820755879367, 0.0110119362210409));
         pointsBeforeIssue.add(new Point3D(0.0382545893376212, 0.0262443294907677, 0.0151346721330367));
         pointsBeforeIssue.add(new Point3D(0.0442214133931064, 0.0188148816283424, 0.0181187342317446));
         pointsBeforeIssue.add(new Point3D(0.0878487223095894, -0.0176017339231975, 0.0359940474533526));
         pointsBeforeIssue.add(new Point3D(0.0317197072764835, 0.0339730913830635, 0.0146770460780876));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(0.0356799421574458, 0.0305305995258588, 0.0121379970533146);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug1Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug1Simplified()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         // pointsBeforeIssue.add(new Point3D(0.1727226445760459, 0.3408246844828842, 0.0707692371805856));
         // pointsBeforeIssue.add(new Point3D(-0.0010094559634557, 0.1846852819495834, -0.0004136019841320));
         // pointsBeforeIssue.add(new Point3D(0.0131019603409140, 0.0679884843470001, 0.0053682349594210));
         pointsBeforeIssue.add(new Point3D(0.0268762363673278, 0.0420820755879367, 0.0110119362210409));
         pointsBeforeIssue.add(new Point3D(0.0382545893376212, 0.0262443294907677, 0.0151346721330367));
         pointsBeforeIssue.add(new Point3D(0.0442214133931064, 0.0188148816283424, 0.0181187342317446));
         // pointsBeforeIssue.add(new Point3D(0.0878487223095894, -0.0176017339231975, 0.0359940474533526));
         pointsBeforeIssue.add(new Point3D(0.0317197072764835, 0.0339730913830635, 0.0146770460780876));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(0.0356799421574458, 0.0305305995258588, 0.0121379970533146);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug2Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug2Original()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.1075136584776464, -0.4824945590361311, 0.0090899461159321));
         pointsBeforeIssue.add(new Point3D(0.5607657881997890, 0.4040329322380422, 0.0474110068485305));
         pointsBeforeIssue.add(new Point3D(0.0289073412127943, 0.0078295230557014, -0.0147523787325835));
         pointsBeforeIssue.add(new Point3D(0.0043375074277710, -0.1974921037188990, 0.0003667227899621));
         pointsBeforeIssue.add(new Point3D(0.0139072921615079, -0.0447432688724709, 0.0011758183858351));
         pointsBeforeIssue.add(new Point3D(0.0186428968993924, -0.0235249738131959, 0.0012995156961376));
         pointsBeforeIssue.add(new Point3D(0.0212072625303128, -0.0130016538284213, 0.0026586776633881));
         pointsBeforeIssue.add(new Point3D(0.0341784903074217, 0.0292151862876490, 0.0028896852699534));
         pointsBeforeIssue.add(new Point3D(0.0643309020398054, 0.0996995467973626, 0.0054389780927482));
         pointsBeforeIssue.add(new Point3D(0.0269036070083187, 0.0083216605291816, 0.0078200543380451));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(0.0325313915792233, 0.0068659303400010, -0.0421477673824605);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug2OriginalV2 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug2OriginalV2()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.0139072921615079, -0.0447432688724709, 0.0011758183858351));
         pointsBeforeIssue.add(new Point3D(0.0186428968993924, -0.0235249738131959, 0.0012995156961376));
         pointsBeforeIssue.add(new Point3D(0.0212072625303128, -0.0130016538284213, 0.0026586776633881));
         pointsBeforeIssue.add(new Point3D(0.0643309020398054, 0.0996995467973626, 0.0054389780927482));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(00.0269036070083187, 0.0083216605291816, 0.0078200543380451);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug2OriginalV3 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug2OriginalV3()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.1075136584776464, -0.4824945590361311, 0.0090899461159321));
         pointsBeforeIssue.add(new Point3D(0.5607657881997890, 0.4040329322380422, 0.0474110068485305));
         pointsBeforeIssue.add(new Point3D(0.0289073412127943, 0.0078295230557014, -0.0147523787325835));
         pointsBeforeIssue.add(new Point3D(0.0043375074277710, -0.1974921037188990, 0.0003667227899621));
         pointsBeforeIssue.add(new Point3D(0.0139072921615079, -0.0447432688724709, 0.0011758183858351));
         pointsBeforeIssue.add(new Point3D(0.0186428968993924, -0.0235249738131959, 0.0012995156961376));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(0.0212072625303128, -0.0130016538284213, 0.0026586776633881);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug2Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug2Simplified()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.1075136584776464, -0.4824945590361311, 0.0090899461159321));
         pointsBeforeIssue.add(new Point3D(0.5607657881997890, 0.4040329322380422, 0.0474110068485305));
         pointsBeforeIssue.add(new Point3D(0.0289073412127943, 0.0078295230557014, -0.0147523787325835));
         pointsBeforeIssue.add(new Point3D(0.0043375074277710, -0.1974921037188990, 0.0003667227899621));
         pointsBeforeIssue.add(new Point3D(0.0139072921615079, -0.0447432688724709, 0.0011758183858351));
         pointsBeforeIssue.add(new Point3D(0.0186428968993924, -0.0235249738131959, 0.0012995156961376));
         pointsBeforeIssue.add(new Point3D(0.0212072625303128, -0.0130016538284213, 0.0026586776633881));
         pointsBeforeIssue.add(new Point3D(0.0341784903074217, 0.0292151862876490, 0.0028896852699534));
         // pointsBeforeIssue.add(new Point3D(0.0643309020398054, 0.0996995467973626, 0.0054389780927482));
         pointsBeforeIssue.add(new Point3D(0.0269036070083187, 0.0083216605291816, 0.0078200543380451));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(0.0325313915792233, 0.0068659303400010, -0.0421477673824605);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug3Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug3Original()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.0037210269862168, 0.0766427555019901, 0.1361514789125481));
         pointsBeforeIssue.add(new Point3D(-0.0098691038563419, -0.0842712352669562, 0.3611081270196839));
         pointsBeforeIssue.add(new Point3D(-0.0013371474796426, 0.1104004526675205, 0.1045058111402412));
         pointsBeforeIssue.add(new Point3D(-0.0036362971658496, 0.0933043645625930, 0.1200567135497745));
         pointsBeforeIssue.add(new Point3D(-0.0024413371416485, 0.1279448469369309, 0.0893279365053837));
         pointsBeforeIssue.add(new Point3D(-0.0064317125888430, 0.1020382766797260, 0.1119023450212495));
         pointsBeforeIssue.add(new Point3D(-0.0012837946563208, 0.1829972596968365, 0.0469737364779878));
         pointsBeforeIssue.add(new Point3D(0.0006330947545868, 0.3027754040393130, -0.0231647841974971));
         pointsBeforeIssue.add(new Point3D(0.0022656588195281, 0.8450024017573801, -0.0828999091198799));
         pointsBeforeIssue.add(new Point3D(-0.0197511024523884, 1.4863939764561370, 0.7226880694514805));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(0.0153396366110251, 0.1027009848979795, 0.1121607898997810);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug3OriginalV2 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug3OriginalV2()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.0037210269862168, 0.0766427555019901, 0.1361514789125481));
         pointsBeforeIssue.add(new Point3D(-0.0098691038563419, -0.0842712352669562, 0.3611081270196839));
         pointsBeforeIssue.add(new Point3D(-0.0013371474796426, 0.1104004526675205, 0.1045058111402412));
         pointsBeforeIssue.add(new Point3D(-0.0036362971658496, 0.0933043645625930, 0.1200567135497745));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(-0.0024413371416485, 0.1279448469369309, 0.0893279365053837);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug3Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug3Simplified()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         //      pointsBeforeIssue.add(new Point3D(-0.0037210269862168, 0.0766427555019901, 0.1361514789125481));
         //      pointsBeforeIssue.add(new Point3D(-0.0098691038563419, -0.0842712352669562, 0.3611081270196839));
         pointsBeforeIssue.add(new Point3D(-0.0013371474796426, 0.1104004526675205, 0.1045058111402412));
         pointsBeforeIssue.add(new Point3D(-0.0036362971658496, 0.0933043645625930, 0.1200567135497745));
         pointsBeforeIssue.add(new Point3D(-0.0024413371416485, 0.1279448469369309, 0.0893279365053837));
         pointsBeforeIssue.add(new Point3D(-0.0064317125888430, 0.1020382766797260, 0.1119023450212495));
         //      pointsBeforeIssue.add(new Point3D(-0.0012837946563208, 0.1829972596968365, 0.0469737364779878));
         //      pointsBeforeIssue.add(new Point3D(0.0006330947545868, 0.3027754040393130, -0.0231647841974971));
         //      pointsBeforeIssue.add(new Point3D(0.0022656588195281, 0.8450024017573801, -0.0828999091198799));
         pointsBeforeIssue.add(new Point3D(-0.0197511024523884, 1.4863939764561370, 0.7226880694514805));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(0.0153396366110251, 0.1027009848979795, 0.1121607898997810);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug4Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug4Original()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.0051506551364596, -0.0348819759911019, 0.0691778540151096));
         pointsBeforeIssue.add(new Point3D(0.0422493845451721, 0.0727920949601139, 0.5674466022012327));
         pointsBeforeIssue.add(new Point3D(0.0004104647457426, -0.0451992694194132, 0.0472529766744278));
         pointsBeforeIssue.add(new Point3D(0.0020291613573142, -0.0559465436341543, 0.0250833552689338));
         pointsBeforeIssue.add(new Point3D(-0.0033350301665630, -0.0940280104643934, -0.0447924048275297));
         pointsBeforeIssue.add(new Point3D(-0.0113077830899894, -0.1649473194674913, -0.1518735281452312));
         pointsBeforeIssue.add(new Point3D(-0.0253624237359933, -0.3387902241779525, -0.3406397827449963));
         pointsBeforeIssue.add(new Point3D(-0.0470958837642707, -1.2942558333752370, -0.6325393732333888));
         pointsBeforeIssue.add(new Point3D(0.0046484599886485, -0.0503176108042466, 0.0362947448605222));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(-0.0068611168376205, -0.0525067429256160, 0.0327580791751562);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug4Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug4Simplified()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.0051506551364596, -0.0348819759911019, 0.0691778540151096));
         // pointsBeforeIssue.add(new Point3D(0.0422493845451721, 0.0727920949601139, 0.5674466022012327));
         pointsBeforeIssue.add(new Point3D(0.0004104647457426, -0.0451992694194132, 0.0472529766744278));
         pointsBeforeIssue.add(new Point3D(0.0020291613573142, -0.0559465436341543, 0.0250833552689338));
         pointsBeforeIssue.add(new Point3D(-0.0033350301665630, -0.0940280104643934, -0.0447924048275297));
         // pointsBeforeIssue.add(new Point3D(-0.0113077830899894, -0.1649473194674913, -0.1518735281452312));
         // pointsBeforeIssue.add(new Point3D(-0.0253624237359933, -0.3387902241779525, -0.3406397827449963));
         // pointsBeforeIssue.add(new Point3D(-0.0470958837642707, -1.2942558333752370, -0.6325393732333888));
         pointsBeforeIssue.add(new Point3D(0.0046484599886485, -0.0503176108042466, 0.0362947448605222));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(-0.0068611168376205, -0.0525067429256160, 0.0327580791751562);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug5 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug5()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.00987877860785552000, 0.19131147056882847000, 0.03418482283725754500));
         pointsBeforeIssue.add(new Point3D(0.02555035291707655200, 0.11906397969616522000, 0.09162654986018806000));
         pointsBeforeIssue.add(new Point3D(0.03548092653292492000, 0.08553875648964482000, 0.12277926611952411000));
         pointsBeforeIssue.add(new Point3D(0.02835400199038940000, 0.11051604727461928000, 0.09913549801006882000));
         pointsBeforeIssue.add(new Point3D(0.19811056289014100000, 1.53064568455729290000, 0.68554775478046680000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(0.03326520528372667000, 0.09790747514458276000, 0.11042444275049243000);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug6 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug6()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.16594466242134853000, -0.15167737932767710000, 0.06375940884903786000));
         pointsBeforeIssue.add(new Point3D(-0.15493261840750883000, -0.16646658348557453000, 0.06032647391633983000));
         pointsBeforeIssue.add(new Point3D(-0.18402867588922667000, -0.12629220803748709000, 0.07363271127979126000));
         pointsBeforeIssue.add(new Point3D(-0.20441775948060736000, -0.10131005633356671000, 0.07902696117881491000));
         pointsBeforeIssue.add(new Point3D(-0.28134730614168046000, -0.01222890247491981200, 0.10876756841830115000));
         pointsBeforeIssue.add(new Point3D(0.08114060867902650000, -1.60769973787097800000, -0.03136858435585743000));
         pointsBeforeIssue.add(new Point3D(-0.13409442245051250000, -0.19646058810895028000, 0.05184028405466168000));
         pointsBeforeIssue.add(new Point3D(-0.17583758138846740000, -0.13878848467476557000, 0.06663279438238556000));
         pointsBeforeIssue.add(new Point3D(-1.32350866683800360000, 0.45061221257020545000, 0.51166233452409130000));
         pointsBeforeIssue.add(new Point3D(-0.45317219159206390000, 0.14566075149172053000, 0.17519427511219820000));

         // Could not reproduce the original exception, but manage to highlight a new one by shuffling the pointsBeforeIssue in a given way
         Random random = new Random(478396840718102216L);
         Collections.shuffle(pointsBeforeIssue, random);
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(-0.16885169989877480000, -0.14496535839630130000, 0.07101749538537033000);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug6V2 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug6V2()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.28134730614168046000, -0.01222890247491981200, 0.10876756841830115000));
         pointsBeforeIssue.add(new Point3D(-0.18402867588922667000, -0.12629220803748709000, 0.07363271127979126000));
         pointsBeforeIssue.add(new Point3D(-0.17583758138846740000, -0.13878848467476557000, 0.06663279438238556000));
         pointsBeforeIssue.add(new Point3D(-0.45317219159206390000, 0.14566075149172053000, 0.17519427511219820000));
         pointsBeforeIssue.add(new Point3D(-0.16594466242134853000, -0.15167737932767710000, 0.06375940884903786000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(-0.20441775948060736000, -0.10131005633356671000, 0.07902696117881491000);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug6V3 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug6V3()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.15493261840750883000, -0.16646658348557453000, 0.06032647391633983000));
         pointsBeforeIssue.add(new Point3D(0.08114060867902650000, -1.60769973787097800000, -0.03136858435585743000));
         pointsBeforeIssue.add(new Point3D(-0.18402867588922667000, -0.12629220803748709000, 0.07363271127979126000));
         pointsBeforeIssue.add(new Point3D(-0.17583758138846740000, -0.13878848467476557000, 0.06663279438238556000));
         pointsBeforeIssue.add(new Point3D(-0.45317219159206390000, 0.14566075149172053000, 0.17519427511219820000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(-1.32350866683800360000, 0.45061221257020545000, 0.51166233452409130000);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug6V4 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug6V4()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.18402867588922667000, -0.12629220803748709000, 0.07363271127979126000));
         pointsBeforeIssue.add(new Point3D(-0.17583758138846740000, -0.13878848467476557000, 0.06663279438238556000));
         pointsBeforeIssue.add(new Point3D(-0.45317219159206390000, 0.14566075149172053000, 0.17519427511219820000));
         pointsBeforeIssue.add(new Point3D(-0.13409442245051250000, -0.19646058810895028000, 0.05184028405466168000));
         pointsBeforeIssue.add(new Point3D(-0.20441775948060736000, -0.10131005633356671000, 0.07902696117881491000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);
         troublesomePoint.set(-0.16885169989877480000, -0.14496535839630130000, 0.07101749538537033000);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug7Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug7Original()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.40484035046382340000, 0.66817675595325240000, 0.37565781894372985000));
         pointsBeforeIssue.add(new Point3D(0.19770492108631288000, -0.11722888133146891000, 0.18345354993551521000));
         pointsBeforeIssue.add(new Point3D(0.13177056177929392000, -0.01608972405889597000, 0.12227200619277923000));
         pointsBeforeIssue.add(new Point3D(0.10804284059622116000, 0.04356860347168567400, 0.10025467521792486000));
         pointsBeforeIssue.add(new Point3D(0.10535595585195023000, 0.05183458577843358400, 0.09786307209807943000));
         pointsBeforeIssue.add(new Point3D(0.10419258778046014000, 0.05592435430920772400, 0.09660853251948309000));
         pointsBeforeIssue.add(new Point3D(0.09875689526332210000, 0.07507951247051448000, 0.09163809842020465000));
         pointsBeforeIssue.add(new Point3D(0.09126555383425328000, 0.10746732352304089000, 0.08468676321119539000));
         pointsBeforeIssue.add(new Point3D(0.09891220007971846000, 0.37479443587413260000, 0.09178220823665961000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-4);

         troublesomePoint.set(0.10219794708629126000, 0.06220152556068970000, 0.09504439824552569000);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug7Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug7Simplified()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         //      pointsBeforeIssue.add(new Point3D( 0.40484035046382340000,  0.66817675595325240000,  0.37565781894372985000 ));
         pointsBeforeIssue.add(new Point3D(0.19770492108631288000, -0.11722888133146891000, 0.18345354993551521000));
         pointsBeforeIssue.add(new Point3D(0.13177056177929392000, -0.01608972405889597000, 0.12227200619277923000));
         pointsBeforeIssue.add(new Point3D(0.10804284059622116000, 0.04356860347168567400, 0.10025467521792486000));
         pointsBeforeIssue.add(new Point3D(0.10535595585195023000, 0.05183458577843358400, 0.09786307209807943000));
         //      pointsBeforeIssue.add(new Point3D( 0.10419258778046014000,  0.05592435430920772400,  0.09660853251948309000 ));
         //      pointsBeforeIssue.add(new Point3D( 0.09875689526332210000,  0.07507951247051448000,  0.09163809842020465000 ));
         //      pointsBeforeIssue.add(new Point3D( 0.09126555383425328000,  0.10746732352304089000,  0.08468676321119539000 ));
         //      pointsBeforeIssue.add(new Point3D( 0.09891220007971846000,  0.37479443587413260000,  0.09178220823665961000 ));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-4);

         troublesomePoint.set(0.10219794708629126000, 0.06220152556068970000, 0.09504439824552569000);
      }
   }

   public static class DatasetEPAFaceNormalIntegrityBug8Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetEPAFaceNormalIntegrityBug8Original()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.25923025651672880000, -1.32459442213312100000, 0.33731064563758340000));
         pointsBeforeIssue.add(new Point3D(-0.58211725366830260000, -0.18145508297583146000, -0.75745150010666570000));
         pointsBeforeIssue.add(new Point3D(-0.29186699291394436000, -0.26463480549211205000, -0.37977759673184330000));
         pointsBeforeIssue.add(new Point3D(-0.15759191168890652000, -0.36628560982904357000, -0.20505873887300530000));
         pointsBeforeIssue.add(new Point3D(-0.14713791925305353000, -0.37644945730058765000, -0.19100593449992065000));
         pointsBeforeIssue.add(new Point3D(-0.14153660446287397000, -0.38162347173657220000, -0.18434170307648423000));
         pointsBeforeIssue.add(new Point3D(-0.12585187210813410000, -0.39752753793187856000, -0.16375857049215160000));
         pointsBeforeIssue.add(new Point3D(-0.09496404628807320000, -0.43099089641061805000, -0.12356730343211364000));
         pointsBeforeIssue.add(new Point3D(-0.03597410587601624000, -0.50432786022543690000, -0.04680953929654707600));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 5.0e-4);

         troublesomePoint.set(-0.13693975331059360000, -0.38652881789871840000, -0.17763839169767880000);
      }
   }

   public static class DatasetEPAFaceNormalIntegrityBug8Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetEPAFaceNormalIntegrityBug8Simplified()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.58211725366830260000, -0.18145508297583146000, -0.75745150010666570000));
         pointsBeforeIssue.add(new Point3D(-0.15759191168890652000, -0.36628560982904357000, -0.20505873887300530000));
         pointsBeforeIssue.add(new Point3D(-0.14713791925305353000, -0.37644945730058765000, -0.19100593449992065000));
         pointsBeforeIssue.add(new Point3D(-0.14153660446287397000, -0.38162347173657220000, -0.18434170307648423000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 5.0e-4);
         troublesomePoint.set(-0.13693975331059360000, -0.38652881789871840000, -0.17763839169767880000);
      }
   }

   public static class DatasetEPAFaceNormalIntegrityBug8SimplifiedV2 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetEPAFaceNormalIntegrityBug8SimplifiedV2()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.58211725366830260000, -0.18145508297583146000, -0.75745150010666570000));
         pointsBeforeIssue.add(new Point3D(-0.15759191168890652000, -0.36628560982904357000, -0.20505873887300530000));
         pointsBeforeIssue.add(new Point3D(-0.14713791925305353000, -0.37644945730058765000, -0.19100593449992065000));
         pointsBeforeIssue.add(new Point3D(-0.14153660446287397000, -0.38162347173657220000, -0.18434170307648423000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 5.0e-4);
         troublesomePoint.set(-0.13693975331059360000, -0.38652881789871840000, -0.17763839169767880000);
      }
   }

   public static class DatasetEPAFaceNormalIntegrityBug9Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetEPAFaceNormalIntegrityBug9Original()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.25923025651672880000, -1.32459442213312100000, 0.33731064563758340000));
         pointsBeforeIssue.add(new Point3D(-0.58211725366830260000, -0.18145508297583146000, -0.75745150010666570000));
         pointsBeforeIssue.add(new Point3D(-0.29186699291394436000, -0.26463480549211205000, -0.37977759673184330000));
         pointsBeforeIssue.add(new Point3D(-0.15759191168890652000, -0.36628560982904357000, -0.20505873887300530000));
         pointsBeforeIssue.add(new Point3D(-0.14713791925305353000, -0.37644945730058765000, -0.19100593449992065000));
         pointsBeforeIssue.add(new Point3D(-0.14153660446287397000, -0.38162347173657220000, -0.18434170307648423000));
         pointsBeforeIssue.add(new Point3D(-0.12585187210813410000, -0.39752753793187856000, -0.16375857049215160000));
         pointsBeforeIssue.add(new Point3D(-0.09496404628807320000, -0.43099089641061805000, -0.12356730343211364000));
         pointsBeforeIssue.add(new Point3D(-0.03597410587601624000, -0.50432786022543690000, -0.04680953929654707600));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 5.0e-4);

         troublesomePoint.set(-0.13693975331059360000, -0.38652881789871840000, -0.17763839169767880000);
      }
   }

   public static class DatasetEPAFaceNormalIntegrityBug9Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetEPAFaceNormalIntegrityBug9Simplified()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         //      pointsBeforeIssue.add(new Point3D( 0.25923025651672880000, -1.32459442213312100000,  0.33731064563758340000 ));
         pointsBeforeIssue.add(new Point3D(-0.58211725366830260000, -0.18145508297583146000, -0.75745150010666570000));
         pointsBeforeIssue.add(new Point3D(-0.29186699291394436000, -0.26463480549211205000, -0.37977759673184330000));
         pointsBeforeIssue.add(new Point3D(-0.15759191168890652000, -0.36628560982904357000, -0.20505873887300530000));
         pointsBeforeIssue.add(new Point3D(-0.14713791925305353000, -0.37644945730058765000, -0.19100593449992065000));
         pointsBeforeIssue.add(new Point3D(-0.14153660446287397000, -0.38162347173657220000, -0.18434170307648423000));
         //      pointsBeforeIssue.add(new Point3D(-0.12585187210813410000, -0.39752753793187856000, -0.16375857049215160000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.09496404628807320000, -0.43099089641061805000, -0.12356730343211364000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.03597410587601624000, -0.50432786022543690000, -0.04680953929654707600 ));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 5.0e-4);

         troublesomePoint.set(-0.13693975331059360000, -0.38652881789871840000, -0.17763839169767880000);
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug10Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug10Original()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.17317378540777761000, -0.05775495286524568000, 0.14175166115938010000));
         pointsBeforeIssue.add(new Point3D(-0.01897404558870108300, 0.00308339916520594670, 0.01553123340681406000));
         pointsBeforeIssue.add(new Point3D(-0.00274787469423487530, 0.01193080363919030200, 0.00214472895451733870));
         pointsBeforeIssue.add(new Point3D(-0.35181526244719350000, 2.19178801525574850000, 0.28797890948495250000));
         pointsBeforeIssue.add(new Point3D(0.52312335359372390000, 1.17120574040232170000, -0.42820340381521560000));
         pointsBeforeIssue.add(new Point3D(0.35783508614818305000, 0.38370415108791240000, -0.29290644518264797000));
         pointsBeforeIssue.add(new Point3D(0.12404154368939402000, 0.09927405350457708000, -0.10153439118595142000));
         pointsBeforeIssue.add(new Point3D(0.05424289486816125000, 0.04693494504728657000, -0.04440060275503699000));
         pointsBeforeIssue.add(new Point3D(0.01802380347325072000, 0.02392344071554553000, -0.01475341131840446400));
         pointsBeforeIssue.add(new Point3D(-0.00038260858106387020, 0.01322913616138221600, 0.00031318545046343793));
         pointsBeforeIssue.add(new Point3D(-0.00965583175989204200, 0.00808735606657740000, 0.00790374855115949500));
         pointsBeforeIssue.add(new Point3D(-0.00501274681955910000, 0.01064106008930820700, 0.00410473368994557200));
         pointsBeforeIssue.add(new Point3D(-0.00129912505345441960, 0.01133059642016243100, 0.00613679617411944100));
         pointsBeforeIssue.add(new Point3D(-0.00280702931313747700, 0.01130231935193704000, 0.00437573895350351600));
         pointsBeforeIssue.add(new Point3D(-0.00386839007299566440, 0.01128503414358750200, 0.00313791547173392100));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-7);

         troublesomePoint.set(-0.00276525936788729600, 0.01161966868636366000, 0.00326143254637401000);
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug10Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug10Simplified()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         //      pointsBeforeIssue.add(new Point3D(-0.17317378540777761000, -0.05775495286524568000, 0.14175166115938010000));
         //      pointsBeforeIssue.add(new Point3D(-0.01897404558870108300, 0.00308339916520594670, 0.01553123340681406000));
         pointsBeforeIssue.add(new Point3D(-0.00274787469423487530, 0.01193080363919030200, 0.00214472895451733870));
         //      pointsBeforeIssue.add(new Point3D(-0.35181526244719350000, 2.19178801525574850000, 0.28797890948495250000));
         //      pointsBeforeIssue.add(new Point3D(0.52312335359372390000, 1.17120574040232170000, -0.42820340381521560000));
         //      pointsBeforeIssue.add(new Point3D(0.35783508614818305000, 0.38370415108791240000, -0.29290644518264797000));
         //      pointsBeforeIssue.add(new Point3D(0.12404154368939402000, 0.09927405350457708000, -0.10153439118595142000));
         //      pointsBeforeIssue.add(new Point3D(0.05424289486816125000, 0.04693494504728657000, -0.04440060275503699000));
         //      pointsBeforeIssue.add(new Point3D(0.01802380347325072000, 0.02392344071554553000, -0.01475341131840446400));
         //      pointsBeforeIssue.add(new Point3D(-0.00038260858106387020, 0.01322913616138221600, 0.00031318545046343793));
         //      pointsBeforeIssue.add(new Point3D(-0.00965583175989204200, 0.00808735606657740000, 0.00790374855115949500));
         pointsBeforeIssue.add(new Point3D(-0.00501274681955910000, 0.01064106008930820700, 0.00410473368994557200));
         //      pointsBeforeIssue.add(new Point3D(-0.00129912505345441960, 0.01133059642016243100, 0.00613679617411944100));
         pointsBeforeIssue.add(new Point3D(-0.00280702931313747700, 0.01130231935193704000, 0.00437573895350351600));
         pointsBeforeIssue.add(new Point3D(-0.00386839007299566440, 0.01128503414358750200, 0.00313791547173392100));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-7);

         troublesomePoint.set(-0.00276525936788729600, 0.01161966868636366000, 0.00326143254637401000);
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug11Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug11Original()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.41935432254164250000, -0.49779783391735880000, 0.51288709338555660000));
         pointsBeforeIssue.add(new Point3D(-0.93298416837861000000, 1.29815954722883200000, 1.14107691890293680000));
         pointsBeforeIssue.add(new Point3D(-0.30925383873489704000, 0.66208031898425670000, 0.37822980220101690000));
         pointsBeforeIssue.add(new Point3D(-0.25512404662567070000, 0.06730540401802787000, 0.31202690348710493000));
         pointsBeforeIssue.add(new Point3D(-0.25507813765207943000, 0.06013191657288902000, 0.31339072703416770000));
         pointsBeforeIssue.add(new Point3D(-0.25531309357128740000, 0.06263846976145082000, 0.31272772115361480000));
         pointsBeforeIssue.add(new Point3D(-0.25641070430508184000, 0.05800263215782730500, 0.31270661888213390000));
         pointsBeforeIssue.add(new Point3D(-0.25641896081503250000, 0.05332514660554210600, 0.31361063528392374000));
         Collections.shuffle(pointsBeforeIssue, new Random(7106315192527041498L));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);

         troublesomePoint.set(-0.25784327149254240000, 0.05771251668497923000, 0.31159424166064453000);
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug11Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug11Simplified()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.41935432254164250000, -0.49779783391735880000, 0.51288709338555660000));
         pointsBeforeIssue.add(new Point3D(-0.93298416837861000000, 1.29815954722883200000, 1.14107691890293680000));
         pointsBeforeIssue.add(new Point3D(-0.30925383873489704000, 0.66208031898425670000, 0.37822980220101690000));
         //      pointsBeforeIssue.add(new Point3D(-0.25512404662567070000,  0.06730540401802787000,  0.31202690348710493000 ));
         pointsBeforeIssue.add(new Point3D(-0.25507813765207943000, 0.06013191657288902000, 0.31339072703416770000));
         pointsBeforeIssue.add(new Point3D(-0.25531309357128740000, 0.06263846976145082000, 0.31272772115361480000));
         pointsBeforeIssue.add(new Point3D(-0.25641070430508184000, 0.05800263215782730500, 0.31270661888213390000));
         pointsBeforeIssue.add(new Point3D(-0.25641896081503250000, 0.05332514660554210600, 0.31361063528392374000));
         Collections.shuffle(pointsBeforeIssue, new Random(7106315192527041498L));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);

         troublesomePoint.set(-0.25784327149254240000, 0.05771251668497923000, 0.31159424166064453000);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug12Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug12Original()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-1.06285928606209360000, 0.84863501402184480000, -0.36386991230254034000));
         pointsBeforeIssue.add(new Point3D(-0.24794687685778950000, -0.88150811138554650000, -0.08488462162493815000));
         pointsBeforeIssue.add(new Point3D(-0.14860460603249515000, 0.25024717144708375000, -0.05087479186933352400));
         pointsBeforeIssue.add(new Point3D(-0.04593802791056822400, -0.33056946184021730000, -0.01572688539900857200));
         pointsBeforeIssue.add(new Point3D(-0.05894221041959524000, -0.03259192667445954000, -0.02017886771799948200));
         pointsBeforeIssue.add(new Point3D(-0.09448585378890406000, 0.11211693061937889000, -0.03234723522001137000));
         pointsBeforeIssue.add(new Point3D(-0.07434099339443390000, 0.04041368632546394000, -0.02545064158692189600));
         pointsBeforeIssue.add(new Point3D(-0.06604318267016518000, 0.00405189569758113160, -0.02260988580670941800));
         pointsBeforeIssue.add(new Point3D(-0.06234252506146254000, -0.01423750400223156000, -0.02134296554921921700));
         pointsBeforeIssue.add(new Point3D(-0.06460601902846741000, -0.00286855106762995400, -0.02211787279677335200));
         pointsBeforeIssue.add(new Point3D(-0.06167537739882600000, -0.01770921927976720800, -0.02111456740931594300));
         pointsBeforeIssue.add(new Point3D(-0.06762831802521285000, 0.01144471242318934000, -0.02315255872024968300));
         pointsBeforeIssue.add(new Point3D(-0.06306822611543828000, -0.01052577856589109700, -0.02159139726896253000));
         pointsBeforeIssue.add(new Point3D(-0.06095078718494506600, -0.02154749334084593600, -0.02086667479886617000));
         pointsBeforeIssue.add(new Point3D(-0.06721513866931716000, 0.00953728141767107300, -0.02300866411810875700));
         pointsBeforeIssue.add(new Point3D(-0.06287435077799386000, -0.01148375396021106300, -0.02154246977939222600));
         pointsBeforeIssue.add(new Point3D(-0.06094046529089814000, -0.02202728685290667000, -0.02060449498172178800));
         pointsBeforeIssue.add(new Point3D(-0.06604214004639730000, 0.00928429102395866800, -0.02627429468814690400));
         pointsBeforeIssue.add(new Point3D(-0.06782601695094026000, -0.00492098617148972700, -0.01152854836332983800));
         pointsBeforeIssue.add(new Point3D(-0.06208347428735250000, -0.00677686010372791100, -0.02694477655902294300));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);

         troublesomePoint.set(-0.06208347428735250000, -0.00677686010372791100, -0.02694477655902294300);
      }
   }

   public static class DatasetGJKNullPointerExceptionBug12Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug12Simplified()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-1.06285928606209360000, 0.84863501402184480000, -0.36386991230254034000));
         //      pointsBeforeIssue.add(new Point3D(-0.24794687685778950000, -0.88150811138554650000, -0.08488462162493815000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.14860460603249515000,  0.25024717144708375000, -0.05087479186933352400 ));
         //      pointsBeforeIssue.add(new Point3D(-0.04593802791056822400, -0.33056946184021730000, -0.01572688539900857200 ));
         pointsBeforeIssue.add(new Point3D(-0.05894221041959524000, -0.03259192667445954000, -0.02017886771799948200));
         //      pointsBeforeIssue.add(new Point3D(-0.09448585378890406000,  0.11211693061937889000, -0.03234723522001137000 ));
         pointsBeforeIssue.add(new Point3D(-0.07434099339443390000, 0.04041368632546394000, -0.02545064158692189600));
         //      pointsBeforeIssue.add(new Point3D(-0.06604318267016518000,  0.00405189569758113160, -0.02260988580670941800 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06234252506146254000, -0.01423750400223156000, -0.02134296554921921700 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06460601902846741000, -0.00286855106762995400, -0.02211787279677335200 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06167537739882600000, -0.01770921927976720800, -0.02111456740931594300 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06762831802521285000,  0.01144471242318934000, -0.02315255872024968300 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06306822611543828000, -0.01052577856589109700, -0.02159139726896253000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06095078718494506600, -0.02154749334084593600, -0.02086667479886617000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06721513866931716000,  0.00953728141767107300, -0.02300866411810875700 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06287435077799386000, -0.01148375396021106300, -0.02154246977939222600 ));
         pointsBeforeIssue.add(new Point3D(-0.06094046529089814000, -0.02202728685290667000, -0.02060449498172178800));
         pointsBeforeIssue.add(new Point3D(-0.06604214004639730000, 0.00928429102395866800, -0.02627429468814690400));
         //      pointsBeforeIssue.add(new Point3D(-0.06782601695094026000, -0.00492098617148972700, -0.01152854836332983800 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06208347428735250000, -0.00677686010372791100, -0.02694477655902294300 ));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);

         troublesomePoint.set(-0.06208347428735250000, -0.00677686010372791100, -0.02694477655902294300);
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug13Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug13Original()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-1.16485579659865150000, 1.50478742011243050000, 0.92998233563902600000));
         pointsBeforeIssue.add(new Point3D(-0.49671459562836695000, -0.57321041192247600000, 0.39656050228474937000));
         pointsBeforeIssue.add(new Point3D(-0.28684916277252004000, 0.75215556219304490000, 0.22901088284939053000));
         pointsBeforeIssue.add(new Point3D(-0.23434852727844335000, 0.04865437022396490500, 0.18709611214396826000));
         pointsBeforeIssue.add(new Point3D(-0.22769504113460493000, 0.09534067425821080000, 0.18178418889796938000));
         pointsBeforeIssue.add(new Point3D(-0.23084361780005236000, 0.07195594672506622000, 0.18429790835562376000));
         pointsBeforeIssue.add(new Point3D(-0.23255161132903990000, 0.06029420820407039000, 0.18566151389030200000));
         pointsBeforeIssue.add(new Point3D(-0.23343896451436674000, 0.05447148220883074600, 0.18636994732063494000));
         pointsBeforeIssue.add(new Point3D(-0.23084251150272717000, 0.07196370955534642000, 0.18429702512443302000));
         pointsBeforeIssue.add(new Point3D(-0.23255102857838360000, 0.06029808252032909000, 0.18566104864145722000));
         pointsBeforeIssue.add(new Point3D(-0.23343866574888150000, 0.05447341752291190400, 0.18636970879614645000));
         pointsBeforeIssue.add(new Point3D(-0.23084361798631325000, 0.07195594541800332000, 0.18429790850435868000));
         pointsBeforeIssue.add(new Point3D(-0.23255161142718340000, 0.06029420755173304000, 0.18566151396861086000));
         pointsBeforeIssue.add(new Point3D(-0.23343896456459690000, 0.05447148188295081000, 0.18636994736089696000));
         pointsBeforeIssue.add(new Point3D(-0.23084251131672584000, 0.07196371086271980000, 0.18429702497529600000));
         pointsBeforeIssue.add(new Point3D(-0.23255102847977416000, 0.06029808317281127000, 0.18566104856369010000));
         pointsBeforeIssue.add(new Point3D(-0.23343866570020700000, 0.05447341784881787000, 0.18636970875392822000));
         pointsBeforeIssue.add(new Point3D(-0.23084361816609666000, 0.07195594411052708000, 0.18429790866132790000));
         pointsBeforeIssue.add(new Point3D(-0.23255161153516546000, 0.06029420689919168000, 0.18566151403466002000));
         pointsBeforeIssue.add(new Point3D(-0.23343896458049573000, 0.05447148155701498000, 0.18636994744417840000));
         pointsBeforeIssue.add(new Point3D(-0.23084251126809020000, 0.07196371217031738000, 0.18429702465403558000));
         pointsBeforeIssue.add(new Point3D(-0.23255102817507890000, 0.06029808382542023400, 0.18566104874401880000));
         pointsBeforeIssue.add(new Point3D(-0.23343866637276056000, 0.05447341817483153400, 0.18636970780829720000));
         pointsBeforeIssue.add(new Point3D(-0.23084361546016796000, 0.07195594280261897000, 0.18429791243294170000));
         pointsBeforeIssue.add(new Point3D(-0.23255161597315000000, 0.06029420624642717000, 0.18566150867720305000));
         pointsBeforeIssue.add(new Point3D(-0.23343894943555432000, 0.05447148123095147000, 0.18636996651731620000));
         pointsBeforeIssue.add(new Point3D(-0.23084257187954450000, 0.07196371347842823000, 0.18429694835241683000));
         pointsBeforeIssue.add(new Point3D(-0.23255093689286900000, 0.06029808447907076000, 0.18566116287855428000));
         pointsBeforeIssue.add(new Point3D(-0.23343898546925080000, 0.05447341850844744600, 0.18636930801633494000));
         pointsBeforeIssue.add(new Point3D(-0.23084233870854210000, 0.07195594146460049000, 0.18429951203091433000));
         pointsBeforeIssue.add(new Point3D(-0.23255353201902773000, 0.06029420592756607000, 0.18565910882543690000));
         pointsBeforeIssue.add(new Point3D(-0.23343224039050580000, 0.05447148421488279000, 0.18637836911135863000));
         pointsBeforeIssue.add(new Point3D(-0.23086941605680810000, 0.07196370167909760000, 0.18426332918815225000));
         pointsBeforeIssue.add(new Point3D(-0.23251065898995815000, 0.06029823265925538000, 0.18571157034205898000));
         pointsBeforeIssue.add(new Point3D(-0.23357967126484824000, 0.05447487912998916000, 0.18619266304844595000));
         pointsBeforeIssue.add(new Point3D(-0.23027969794173497000, 0.07195015744675376000, 0.18500649211601283000));
         pointsBeforeIssue.add(new Point3D(-0.23190458293936178000, 0.06320641868777910000, 0.18557954893731465000));
         pointsBeforeIssue.add(new Point3D(-0.23378371205299409000, 0.05595052063483186000, 0.18547193991612554000));
         pointsBeforeIssue.add(new Point3D(-0.23118909226302153000, 0.05911705963689206600, 0.18773635485865492000));
         pointsBeforeIssue.add(new Point3D(-0.23349476843202010000, 0.05976000785206925000, 0.18464643281306692000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);

         troublesomePoint.set(-0.23349476843202010000, 0.05976000785206925000, 0.18464643281306692000);
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug13Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug13Simplified()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-1.16485579659865150000, 1.50478742011243050000, 0.92998233563902600000));
         pointsBeforeIssue.add(new Point3D(-0.49671459562836695000, -0.57321041192247600000, 0.39656050228474937000));
         pointsBeforeIssue.add(new Point3D(-0.28684916277252004000, 0.75215556219304490000, 0.22901088284939053000));
         pointsBeforeIssue.add(new Point3D(-0.23434852727844335000, 0.04865437022396490500, 0.18709611214396826000));
         pointsBeforeIssue.add(new Point3D(-0.22769504113460493000, 0.09534067425821080000, 0.18178418889796938000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084361780005236000, 0.07195594672506622000, 0.18429790835562376000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255161132903990000, 0.06029420820407039000, 0.18566151389030200000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343896451436674000, 0.05447148220883074600, 0.18636994732063494000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084251150272717000, 0.07196370955534642000, 0.18429702512443302000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255102857838360000, 0.06029808252032909000, 0.18566104864145722000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343866574888150000, 0.05447341752291190400, 0.18636970879614645000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084361798631325000, 0.07195594541800332000, 0.18429790850435868000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255161142718340000, 0.06029420755173304000, 0.18566151396861086000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343896456459690000, 0.05447148188295081000, 0.18636994736089696000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084251131672584000, 0.07196371086271980000, 0.18429702497529600000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255102847977416000, 0.06029808317281127000, 0.18566104856369010000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343866570020700000, 0.05447341784881787000, 0.18636970875392822000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084361816609666000, 0.07195594411052708000, 0.18429790866132790000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255161153516546000, 0.06029420689919168000, 0.18566151403466002000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343896458049573000, 0.05447148155701498000, 0.18636994744417840000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084251126809020000, 0.07196371217031738000, 0.18429702465403558000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255102817507890000, 0.06029808382542023400, 0.18566104874401880000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343866637276056000, 0.05447341817483153400, 0.18636970780829720000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084361546016796000, 0.07195594280261897000, 0.18429791243294170000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255161597315000000, 0.06029420624642717000, 0.18566150867720305000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343894943555432000, 0.05447148123095147000, 0.18636996651731620000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084257187954450000, 0.07196371347842823000, 0.18429694835241683000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255093689286900000, 0.06029808447907076000, 0.18566116287855428000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343898546925080000, 0.05447341850844744600, 0.18636930801633494000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084233870854210000, 0.07195594146460049000, 0.18429951203091433000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255353201902773000, 0.06029420592756607000, 0.18565910882543690000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343224039050580000, 0.05447148421488279000, 0.18637836911135863000));
         //      pointsBeforeIssue.add(new Point3D(-0.23086941605680810000, 0.07196370167909760000, 0.18426332918815225000));
         //      pointsBeforeIssue.add(new Point3D(-0.23251065898995815000, 0.06029823265925538000, 0.18571157034205898000));
         //      pointsBeforeIssue.add(new Point3D(-0.23357967126484824000, 0.05447487912998916000, 0.18619266304844595000));
         //      pointsBeforeIssue.add(new Point3D(-0.23027969794173497000, 0.07195015744675376000, 0.18500649211601283000));
         pointsBeforeIssue.add(new Point3D(-0.23190458293936178000, 0.06320641868777910000, 0.18557954893731465000));
         pointsBeforeIssue.add(new Point3D(-0.23378371205299409000, 0.05595052063483186000, 0.18547193991612554000));
         pointsBeforeIssue.add(new Point3D(-0.23118909226302153000, 0.05911705963689206600, 0.18773635485865492000));
         //      pointsBeforeIssue.add(new Point3D(-0.23349476843202010000, 0.05976000785206925000, 0.18464643281306692000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-3);

         troublesomePoint.set(-0.23349476843202010000, 0.05976000785206925000, 0.18464643281306692000);
      }
   }

   public static class DatasetGJKFaceNormalIntegrity_20190228_220911 extends ConvexPolytope3DTroublesomeDataset
   {
      /**
       * The main thing this dataset reveals is that new faces are created to only have 3 vertices which
       * is a limitation and would definitely prevent this bug.
       */
      public DatasetGJKFaceNormalIntegrity_20190228_220911()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.69290152806624020000, 0.50768999063042180000, 0.06363179078759229000));
         pointsBeforeIssue.add(new Point3D(0.18276181406493613000, -0.50024222323255470000, 0.01678371463402106300));
         pointsBeforeIssue.add(new Point3D(0.12994643325986270000, 0.16086647763109743000, 0.01193347672051159300));
         pointsBeforeIssue.add(new Point3D(0.06342912096006117000, -0.17717417522993928000, 0.00582493816406426400));
         pointsBeforeIssue.add(new Point3D(0.07338575205979947000, -0.00352996758700958870, 0.00673929358316116560));
         pointsBeforeIssue.add(new Point3D(0.06245096874381456000, -0.09000765157352504000, 0.00573511071432963900));
         pointsBeforeIssue.add(new Point3D(0.06643690653972065000, -0.04657990542776806000, 0.00610115458874538300));
         pointsBeforeIssue.add(new Point3D(0.08928878518501723000, 0.05989188268082375000, 0.00819972978617866700));
         pointsBeforeIssue.add(new Point3D(0.07558237236266321000, 0.00714957099078997200, 0.00694101760581999900));
         pointsBeforeIssue.add(new Point3D(0.06278639727126109000, -0.08456553041509965000, 0.00576591439572837800));
         pointsBeforeIssue.add(new Point3D(0.06751492505399428000, -0.03847309446811371000, 0.00620015314159383400));
         pointsBeforeIssue.add(new Point3D(0.09049802033207752000, 0.06379683716838570000, 0.00831077846327110700));
         pointsBeforeIssue.add(new Point3D(0.07689823545984276000, 0.01313965058830352000, 0.00706185833392369840));
         pointsBeforeIssue.add(new Point3D(0.06299554871702032000, -0.08150567262327739000, 0.00578512157253641100));
         pointsBeforeIssue.add(new Point3D(0.06816703680253616000, -0.03391935874795497000, 0.00626003904916216900));
         pointsBeforeIssue.add(new Point3D(0.09118922701245419000, 0.06598990756695286000, 0.00837425462676999500));
         pointsBeforeIssue.add(new Point3D(0.07766374510111382000, 0.01650336615144861000, 0.00713215799506905500));
         pointsBeforeIssue.add(new Point3D(0.06311967180957340000, -0.07978496015632038000, 0.00579652034065247800));
         pointsBeforeIssue.add(new Point3D(0.06854830123360611000, -0.03136000744856276600, 0.00629505190098900200));
         pointsBeforeIssue.add(new Point3D(0.09158151048933016000, 0.06722239871967348000, 0.00841028044705882700));
         pointsBeforeIssue.add(new Point3D(0.07810234996807222000, 0.01839360256423040700, 0.00717243643491638500));
         pointsBeforeIssue.add(new Point3D(0.06319153925915186000, -0.07881721039423031000, 0.00580312250506209700));
         pointsBeforeIssue.add(new Point3D(0.06876733232926746000, -0.02992106557740965000, 0.00631516224016659400));
         pointsBeforeIssue.add(new Point3D(0.09180326716383555000, 0.06791532097154179000, 0.00843067884245729500));
         pointsBeforeIssue.add(new Point3D(0.07835159687695523000, 0.01945626539304043400, 0.00719531862188327300));
         pointsBeforeIssue.add(new Point3D(0.06323260822286730000, -0.07827290003949239000, 0.00580694502532530500));
         pointsBeforeIssue.add(new Point3D(0.06889199109869065000, -0.02911188382893079000, 0.00632651417617557900));
         pointsBeforeIssue.add(new Point3D(0.09192828327024699000, 0.06830497727632491000, 0.00844294715850857400));
         pointsBeforeIssue.add(new Point3D(0.07849260822028062000, 0.02005382338668646000, 0.00720816013487235450));
         pointsBeforeIssue.add(new Point3D(0.06325584682864169000, -0.07796674112778025000, 0.00580985143225110100));
         pointsBeforeIssue.add(new Point3D(0.06896269980927255000, -0.02865679034622081700, 0.00633151548305721500));
         pointsBeforeIssue.add(new Point3D(0.09199763627676469000, 0.06852412313249402000, 0.00846160465054612800));
         pointsBeforeIssue.add(new Point3D(0.07857228186350942000, 0.02038989034088247000, 0.00721430334882267040));
         pointsBeforeIssue.add(new Point3D(0.06326826137164637000, -0.07779453119896886000, 0.00581935590873783300));
         pointsBeforeIssue.add(new Point3D(0.06900403467768462000, -0.02840082445718784800, 0.00631891665373540700));
         pointsBeforeIssue.add(new Point3D(0.09202500485407222000, 0.06864736217071214000, 0.00859938720213293500));
         pointsBeforeIssue.add(new Point3D(0.07861803431238706000, 0.02057902172424973300, 0.00720874120642356300));
         pointsBeforeIssue.add(new Point3D(0.06326937390894047000, -0.07769760267388713000, 0.00588895865962191200));
         pointsBeforeIssue.add(new Point3D(0.06903900103459407000, -0.02825692897117966500, 0.00618482993385105500));
         pointsBeforeIssue.add(new Point3D(0.09194493958676875000, 0.06871527603606316000, 0.00972599825232489700));
         pointsBeforeIssue.add(new Point3D(0.07865135114561148000, 0.02069355532833770700, 0.00714465985551715500));
         pointsBeforeIssue.add(new Point3D(0.06323064952435398000, -0.07763869526608258000, 0.00636294866345932200));
         pointsBeforeIssue.add(new Point3D(0.06913858511331608000, -0.02817792609644004200, 0.00524622987493783550));
         pointsBeforeIssue.add(new Point3D(0.09128845469645783000, 0.06868421363121194000, 0.01749622914526949300));
         pointsBeforeIssue.add(new Point3D(0.07879569557827140000, 0.02115722735664338200, 0.00678618050052637300));
         pointsBeforeIssue.add(new Point3D(0.06301737621497602000, -0.07739422362078208000, 0.00897688092849968600));
         pointsBeforeIssue.add(new Point3D(0.06966918356725210000, -0.02809655780659242600, -0.00004337270235066359));
         pointsBeforeIssue.add(new Point3D(0.07492645441585288000, 0.01477008624690665000, 0.04604939960311582600));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-2);
         troublesomePoint.set(0.08045526764839975000, 0.02285563605061801000, -0.00538019769538478100);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190302_160115 extends ConvexPolytope3DTroublesomeDataset
   {
      /**
       * This data highlighted a bug in Face3D which was caused by poor management of the
       * constructionEpsilon.
       */
      public ConvexPolytope3DTroublesomeDataset_20190302_160115()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.97083141752365850000, 0.19270232253646270000, -1.10314697851813780000));
         pointsBeforeIssue.add(new Point3D(0.08735078445224298000, -1.63277627043767500000, 0.09925590808079998000));
         pointsBeforeIssue.add(new Point3D(-0.16861590228562445000, -0.35729391130264250000, -0.19159672810234674000));
         pointsBeforeIssue.add(new Point3D(-0.35048289420867120000, -0.10822871051009664000, -0.39825054977597630000));
         pointsBeforeIssue.add(new Point3D(-0.25480704371353974000, -0.22482735589148084000, -0.28953494427973680000));
         pointsBeforeIssue.add(new Point3D(-0.21045139648732920000, -0.28918214661603340000, -0.23913402262165895000));
         pointsBeforeIssue.add(new Point3D(-0.18920976292038977000, -0.32278223301503760000, -0.21499734609348364000));
         pointsBeforeIssue.add(new Point3D(-0.26251174735750926000, -0.21437817652630936000, -0.29828972949971044000));
         pointsBeforeIssue.add(new Point3D(-0.21408984752431032000, -0.28361727533457450000, -0.24326836169998278000));
         pointsBeforeIssue.add(new Point3D(-0.19097303137237798000, -0.31991844716892740000, -0.21700093212295768000));
         pointsBeforeIssue.add(new Point3D(-0.25532530152410793000, -0.22411807334799222000, -0.29012383595290490000));
         pointsBeforeIssue.add(new Point3D(-0.21069657245417367000, -0.28880546074366414000, -0.23941261386019497000));
         pointsBeforeIssue.add(new Point3D(-0.18932870793247086000, -0.32258861213242257000, -0.21513250223734293000));
         pointsBeforeIssue.add(new Point3D(-0.26202498230546567000, -0.21503232293794655000, -0.29773662276369195000));
         pointsBeforeIssue.add(new Point3D(-0.21386038394086704000, -0.28396664073018063000, -0.24300762430112366000));
         pointsBeforeIssue.add(new Point3D(-0.19086194785992644000, -0.32009845157381284000, -0.21687470892949090000));
         pointsBeforeIssue.add(new Point3D(-0.25577615488337163000, -0.22350179846019480000, -0.29063613655908904000));
         pointsBeforeIssue.add(new Point3D(-0.21090980920004387000, -0.28847804640993530000, -0.23965491284606855000));
         pointsBeforeIssue.add(new Point3D(-0.18943214279826670000, -0.32242029058484610000, -0.21525003435633305000));
         pointsBeforeIssue.add(new Point3D(-0.26160192186689590000, -0.21560151221163665000, -0.29725590295121050000));
         pointsBeforeIssue.add(new Point3D(-0.21366090718484270000, -0.28427052340751540000, -0.24278096073773303000));
         pointsBeforeIssue.add(new Point3D(-0.19076536817251855000, -0.32025499828839680000, -0.21676496605945483000));
         pointsBeforeIssue.add(new Point3D(-0.25616834945327020000, -0.22296627726528990000, -0.29108178461917344000));
         pointsBeforeIssue.add(new Point3D(-0.21109526438785820000, -0.28819344177455190000, -0.23986564255056098000));
         pointsBeforeIssue.add(new Point3D(-0.18952208758489730000, -0.32227395694270733000, -0.21535224252176488000));
         pointsBeforeIssue.add(new Point3D(-0.26123421207556220000, -0.21609673816089336000, -0.29683805997600940000));
         pointsBeforeIssue.add(new Point3D(-0.21348747937756680000, -0.28453483575594035000, -0.24258392070106327000));
         pointsBeforeIssue.add(new Point3D(-0.19068143614879580000, -0.32039114237603810000, -0.21666951473614948000));
         pointsBeforeIssue.add(new Point3D(-0.25650935309177320000, -0.22250088499806486000, -0.29146956412627790000));
         pointsBeforeIssue.add(new Point3D(-0.21125677344005106000, -0.28794603693798404000, -0.24004871998837785000));
         pointsBeforeIssue.add(new Point3D(-0.18959954084944286000, -0.32214673477981360000, -0.21544179390074947000));
         pointsBeforeIssue.add(new Point3D(-0.26091744042974535000, -0.21652758125062954000, -0.29647234178060020000));
         pointsBeforeIssue.add(new Point3D(-0.21333273792329100000, -0.28476472339561754000, -0.24241610814400594000));
         pointsBeforeIssue.add(new Point3D(-0.19062152548312090000, -0.32050954055667260000, -0.21657502502991666000));
         pointsBeforeIssue.add(new Point3D(-0.25675701987444710000, -0.22209640957841460000, -0.29184993243613800000));
         pointsBeforeIssue.add(new Point3D(-0.21146931819101095000, -0.28773093222510954000, -0.24014454369490168000));
         pointsBeforeIssue.add(new Point3D(-0.18941766249809594000, -0.32203584875844776000, -0.21573941078587810000));
         pointsBeforeIssue.add(new Point3D(-0.26157550750578480000, -0.21690377434253066000, -0.29533185660364370000));
         pointsBeforeIssue.add(new Point3D(-0.21190304601336580000, -0.28495523356869720000, -0.24342425492789554000));
         pointsBeforeIssue.add(new Point3D(-0.19493169094055962000, -0.32052879613810250000, -0.21277662508582130000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-2);
         troublesomePoint.set(-0.24069598189087021000, -0.22216401323722800000, -0.30617162217711513000);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_111711 extends ConvexPolytope3DTroublesomeDataset
   {
      /**
       * This dataset highlighted a bug when destroying faces, newly faces were also destroyed.
       */
      public ConvexPolytope3DTroublesomeDataset_20190303_111711()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.42651861137013536000, 0.29801909203948130000, -0.50881487394525300000));
         pointsBeforeIssue.add(new Point3D(-0.20516265604483996000, -1.22116838270320940000, 0.24474854834209914000));
         pointsBeforeIssue.add(new Point3D(-0.15474743628494347000, -0.19414762426986742000, 0.18460577144275080000));
         pointsBeforeIssue.add(new Point3D(0.09567299040224675000, 0.16701603647134378000, -0.11413298096209190000));
         pointsBeforeIssue.add(new Point3D(1.25251552911340270000, -0.77979707379684860000, 0.18358441780536028000));
         pointsBeforeIssue.add(new Point3D(-0.42349443545899990000, -0.75484408613651910000, -1.17104055750426640000));
         pointsBeforeIssue.add(new Point3D(0.66343852027333500000, -0.21204323532349700000, 0.37617790738248347000));
         pointsBeforeIssue.add(new Point3D(-0.48038709090787757000, -0.20046079757003776000, -0.56864140285513080000));
         pointsBeforeIssue.add(new Point3D(0.26413164476960427000, -0.03607115324095866400, 0.25419883203236570000));
         pointsBeforeIssue.add(new Point3D(-0.28904627974853190000, -0.03294277079647378600, -0.20572540856203970000));
         pointsBeforeIssue.add(new Point3D(0.02663658174361427400, 0.00806765192311997800, 0.10847859247578451000));
         pointsBeforeIssue.add(new Point3D(-0.10798349555768538000, 0.05234735820899627500, -0.08040989111998509000));
         pointsBeforeIssue.add(new Point3D(0.01968789268608772700, 0.08783884515805285000, -0.01547486359495420200));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-2);
         troublesomePoint.set(-0.12185888684659330000, -0.04755688773206634400, 0.06109223839806776600);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_120656 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190303_120656()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.90845250809078680000, 2.27040165191556880000, 0.85520029030598930000));
         pointsBeforeIssue.add(new Point3D(-0.35028001093434450000, -0.71632092412883900000, 0.32974708569960800000));
         pointsBeforeIssue.add(new Point3D(0.21414664741334005000, 1.07437997978164650000, -0.20159366990006766000));
         pointsBeforeIssue.add(new Point3D(0.15515353708065260000, 0.04631896555688220000, -0.14605865333811735000));
         pointsBeforeIssue.add(new Point3D(0.18259557865711085000, 0.67866202579018810000, 2.01418653258046800000));
         pointsBeforeIssue.add(new Point3D(-1.99950059483868440000, 0.67866202579018850000, -0.30378595196848980000));
         pointsBeforeIssue.add(new Point3D(0.46399006455839110000, -0.09965128578182691000, 1.06559233195128210000));
         pointsBeforeIssue.add(new Point3D(-1.03565694775630700000, -0.09965128578182703000, -0.52743576337442480000));
         pointsBeforeIssue.add(new Point3D(0.24013142981869978000, -0.34191292573890190000, 0.43948795795885110000));
         pointsBeforeIssue.add(new Point3D(-0.42419935030018024000, -0.34191292573890190000, -0.26620984200911800000));
         pointsBeforeIssue.add(new Point3D(-0.03598353834151968000, -0.36912442321079680000, 0.03387423355853442000));
         pointsBeforeIssue.add(new Point3D(0.23524026643487567000, -0.16034770641498058000, 0.13293680366100574000));
         pointsBeforeIssue.add(new Point3D(-0.11850174993658597000, -0.16034770641498020000, -0.24283227129424856000));
         pointsBeforeIssue.add(new Point3D(0.07132931048778474000, -0.17159446392155820000, -0.06714808588583293000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-2);
         troublesomePoint.set(0.20140763882017088000, -0.06242105790225444000, -0.01234876003148449600);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_122006 extends ConvexPolytope3DTroublesomeDataset
   {
      /**
       * This data highlights an edge-case triggered during the expansion of a new face which would result
       * in rejecting the vertex being added.
       */
      public ConvexPolytope3DTroublesomeDataset_20190303_122006()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.72399987695997490000, 0.78925182235832480000, 0.68035162188417670000));
         pointsBeforeIssue.add(new Point3D(-0.10066480308695103000, -0.98268929676327770000, 0.09459595812976185000));
         pointsBeforeIssue.add(new Point3D(-0.00707474541866726100, 0.17173375187195128000, 0.00664822560498046200));
         pointsBeforeIssue.add(new Point3D(0.06720565503243336000, -0.42396129483605040000, -0.06315398366239744000));
         pointsBeforeIssue.add(new Point3D(-1.47209228609775970000, -0.30317489537983360000, -0.11573498208463157000));
         pointsBeforeIssue.add(new Point3D(0.02409253217780982200, -0.30317489537983240000, 1.47643822585298510000));
         pointsBeforeIssue.add(new Point3D(0.36013545166180660000, -0.17032436693446074000, 0.70028801960523150000));
         pointsBeforeIssue.add(new Point3D(-0.67657142012428340000, -0.17032436693445230000, -0.40292923375859380000));
         pointsBeforeIssue.add(new Point3D(0.26227976702423017000, -0.12906871934200104000, 0.24407689541467560000));
         pointsBeforeIssue.add(new Point3D(-0.22731783509811143000, -0.12906871934199704000, -0.27693105443749210000));
         pointsBeforeIssue.add(new Point3D(0.09636164594442587000, -0.11914561874941587000, -0.01716994101479385500));
         pointsBeforeIssue.add(new Point3D(-0.07201500779408909000, -0.00387022171032114230, -0.14349822871148532000));
         pointsBeforeIssue.add(new Point3D(0.03578107824730514000, 0.02537458904681466600, -0.03251882217663682000));
         pointsBeforeIssue.add(new Point3D(0.15793238320759340000, 0.05596883793175411000, 0.14499435712828979000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-2);
         troublesomePoint.set(0.01219650720536491500, -0.07685272305708180000, -0.09436722510817275000);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_142536 extends ConvexPolytope3DTroublesomeDataset
   {
      /**
       * This data highlights the following issue: the vertex is on the support line of an edge. The
       * edge's face is an in-plane face, but the twin's face is not. This would create a broken polytope
       * when poorly handled.
       */
      public ConvexPolytope3DTroublesomeDataset_20190303_142536()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.77952021930564740000, 0.68175091439513100000, 0.88022153629260390000));
         pointsBeforeIssue.add(new Point3D(0.00755257946139870100, -1.37890050133029280000, -0.00852824972571714000));
         pointsBeforeIssue.add(new Point3D(0.00882239922256677900, -0.00550188742598167750, -0.00996210951961151700));
         pointsBeforeIssue.add(new Point3D(0.24782431072536026000, -0.69054427019503040000, 1.79003321475398370000));
         pointsBeforeIssue.add(new Point3D(-1.80686474933665500000, -0.69054427019503040000, -0.02959014216877598000));
         pointsBeforeIssue.add(new Point3D(0.23712437564132222000, 0.20769595225738152000, 1.08708464074358800000));
         pointsBeforeIssue.add(new Point3D(-1.10777895062942820000, 0.20769595225738152000, -0.10395567423794305000));
         pointsBeforeIssue.add(new Point3D(-0.50044462323115240000, 0.48121404765904590000, 0.22273136156388870000));
         pointsBeforeIssue.add(new Point3D(0.04037873351940657000, 0.34514418372584005000, 0.50832207780856380000));
         pointsBeforeIssue.add(new Point3D(-0.15433294366209777000, 0.30325456009572970000, 0.16979995093961175000));
         pointsBeforeIssue.add(new Point3D(0.11718757317754325000, 0.14369068070229263000, 0.26122133011282433000));
         pointsBeforeIssue.add(new Point3D(-0.02241124340936162500, 0.15722149478681585000, 0.11127857352399390000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-2);
         troublesomePoint.set(-0.07807324306319463000, 0.15742988410454184000, 0.06040079563670270000);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_154201 extends ConvexPolytope3DTroublesomeDataset
   {
      /**
       * This data highlights an edge-case triggered going from single face to tetrahedron with the
       * original face big enough to have made it through the epsilon tests, but small enough to trigger
       * new face expansion resulting a polytope with 3 faces instead of 4.
       */
      public ConvexPolytope3DTroublesomeDataset_20190303_154201()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.00513254314630662300, 0.01135304298811235000, 0.01418412475941843700));
         pointsBeforeIssue.add(new Point3D(0.00021120765251825270, -0.01763511715856947000, 0.00058368641199196820));
         pointsBeforeIssue.add(new Point3D(-0.00037900022155279434, 0.00133264215432005170, -0.00104739234977868630));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-2);
         troublesomePoint.set(0.02215470614908077200, -0.00674925963584982500, 0.00802463395282543300);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_165341 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190303_165341()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.87891217943452820000, 0.79872975655805370000, -0.77025592772239560000));
         pointsBeforeIssue.add(new Point3D(0.10414943353234363000, -1.15315213972416180000, -0.09127387289004246000));
         pointsBeforeIssue.add(new Point3D(0.04898639323837373000, 0.13336066496869303000, -0.04293040949082477000));
         pointsBeforeIssue.add(new Point3D(-0.05799225023803511500, -0.52009668762978620000, 0.05082290990272153600));
         pointsBeforeIssue.add(new Point3D(-0.03803986464642317000, -0.18366082728861400000, 0.03333715463190490000));
         pointsBeforeIssue.add(new Point3D(-0.00262433603527673100, -0.02121994150400774400, 0.00229990030233562240));
         pointsBeforeIssue.add(new Point3D(0.02120909388714043000, 0.05723440083901532000, -0.01858710188571258000));
         pointsBeforeIssue.add(new Point3D(0.00879211227490628300, 0.01827592444549197600, -0.00770518450552462600));
         pointsBeforeIssue.add(new Point3D(0.00295828990973345800, -0.00140765941618214980, -0.00259193454719702800));
         pointsBeforeIssue.add(new Point3D(0.00004159554989802139, -0.01129806815557932200, -0.00022538704045779734));
         pointsBeforeIssue.add(new Point3D(0.00165459812629076270, -0.00635201355375258900, -0.00121449960979916340));
         pointsBeforeIssue.add(new Point3D(0.00156308424995355020, -0.00391360267100382400, -0.00272798489336986700));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-4);
         troublesomePoint.set(0.00014363976288112035, -0.01196889612053064800, 0.00027199536220018360);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_172836 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190303_172836()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.06747410667733611000, 0.01097006791540566800, -0.01855584631406093000));
         pointsBeforeIssue.add(new Point3D(-0.23450880249566974000, -0.35514590183382583000, 0.06449154368524224000));
         pointsBeforeIssue.add(new Point3D(-0.16480266637687052000, -0.09997082136786029000, 0.04532187382725045000));
         pointsBeforeIssue.add(new Point3D(-0.06038812163569981500, -0.01809778776209314300, 0.01660715138661117400));
         pointsBeforeIssue.add(new Point3D(0.00202511726880032200, 0.00361784865624836800, -0.00055692126444689680));
         pointsBeforeIssue.add(new Point3D(0.15153572105567930000, -0.30604894915001346000, 0.28711505869528310000));
         pointsBeforeIssue.add(new Point3D(-0.07369353077212262000, -0.25285670143166930000, -0.29738077364433600000));
         pointsBeforeIssue.add(new Point3D(0.05779532695296065000, -0.09290449420847424000, 0.21591551597107142000));
         pointsBeforeIssue.add(new Point3D(-0.06095504775507306000, -0.07133398996452434000, -0.18860259696650689000));
         pointsBeforeIssue.add(new Point3D(0.00884534586488244800, -0.02740019291422951500, 0.12079477706083219000));
         pointsBeforeIssue.add(new Point3D(-0.05169272294775219000, -0.02161279109018687800, -0.09201601161740780000));
         pointsBeforeIssue.add(new Point3D(-0.01341711532192288400, -0.01019306955203092000, 0.06161251335417228000));
         pointsBeforeIssue.add(new Point3D(-0.04100666320760343000, -0.00873109999379462200, -0.03686093630059778000));
         pointsBeforeIssue.add(new Point3D(-0.02491492804702956400, -0.00595864339012780500, 0.02516064751564628400));
         pointsBeforeIssue.add(new Point3D(-0.02883833506108163200, -0.00423312880939052950, -0.00705883193530876250));
         pointsBeforeIssue.add(new Point3D(-0.01562781722835876500, -0.00126061520025416700, 0.00788652333063244400));
         pointsBeforeIssue.add(new Point3D(-0.01257580112268012800, 0.00054105599367043620, -0.00877636839705048200));
         pointsBeforeIssue.add(new Point3D(-0.00817348912662128900, 0.00120252498154777000, 0.00080984691360735940));
         pointsBeforeIssue.add(new Point3D(-0.00064066905671167350, 0.00154563211405506220, 0.01675715532566424000));
         pointsBeforeIssue.add(new Point3D(-0.00200694895364611500, 0.00215160495631450100, 0.00763305473354353200));
         pointsBeforeIssue.add(new Point3D(-0.00281060462236036600, 0.00239234492507844900, 0.00206173472556359100));
         pointsBeforeIssue.add(new Point3D(-0.00408405263766370400, 0.00256756996996077000, -0.00687975352391062560));
         pointsBeforeIssue.add(new Point3D(-0.00215324445034560660, 0.00281684027960105170, -0.00258671120635789450));
         pointsBeforeIssue.add(new Point3D(-0.00078689974031906360, 0.00297654768486216300, 0.00001118064050609213));
         pointsBeforeIssue.add(new Point3D(0.00151196108896307910, 0.00318524073757953100, 0.00434106035620457350));
         pointsBeforeIssue.add(new Point3D(0.00104652179994238480, 0.00326303258118582070, 0.00180419067017462130));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-5);
         troublesomePoint.set(0.00244062696426103340, 0.00376191324193225760, -0.00155780991298043460);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_180109 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190303_180109()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.00018629439324591157, 0.00049034938467718000, -0.00087149249641926830));
         pointsBeforeIssue.add(new Point3D(0.00004798972671832269, -0.00140551706489366880, 0.00022449782847200340));
         pointsBeforeIssue.add(new Point3D(0.00004389856088266031, -0.00013788428156769061, 0.00020535919383435353));
         pointsBeforeIssue.add(new Point3D(-0.00005378436125835773, 0.00032224543715053680, -0.00025160535668600215));
         pointsBeforeIssue.add(new Point3D(0.00106464605894840410, -0.00078885291272845490, -0.00113889949474199610));
         pointsBeforeIssue.add(new Point3D(-0.00143723484544022730, -0.00078885291272867700, -0.00060408549809654040));
         pointsBeforeIssue.add(new Point3D(0.00081800251973424000, -0.00013347834258559121, -0.00042626176850790730));
         pointsBeforeIssue.add(new Point3D(-0.00092078735873357690, -0.00013347834258581326, -0.00005456974967532613));
         pointsBeforeIssue.add(new Point3D(0.00041022156481829164, 0.00006978320198436361, -0.00013442312928102496));
         pointsBeforeIssue.add(new Point3D(-0.00042932789446292750, 0.00006978320198414156, 0.00004504296797114282));
         pointsBeforeIssue.add(new Point3D(0.00009218181317455221, 0.00012020073833973743, -0.00001567068107144465));
         pointsBeforeIssue.add(new Point3D(-0.00015293051115145007, 0.00003934793020243799, 0.00010284252441705721));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-5);
         troublesomePoint.set(-0.00034597504967026627, 0.00038883490048968740, -0.00039829906847554940);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190317_143836 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190317_143836()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.70748115689297640000, 0.96081283193249120000, 1.33209407268086260000));
         pointsBeforeIssue.add(new Point3D(0.02109978554369074000, -1.70635128569457660000, 0.03972812418216753500));
         pointsBeforeIssue.add(new Point3D(-0.00600275773596248300, 0.06035648754052997000, -0.01130240420102690500));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-6);
         troublesomePoint.set(-0.82482261969415400000, -0.77419355574449570000, 2.14590750433259150000);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190317_161948 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190317_161948()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.01322660245066220600, 0.01363953726236516500, -0.00822103454813905000));
         pointsBeforeIssue.add(new Point3D(0.01358157291376119200, 0.01302508759426573800, -0.00864517544450316100));
         pointsBeforeIssue.add(new Point3D(0.01412701874723776300, 0.01241423690826731800, -0.00875830460347137900));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-6);
         troublesomePoint.set(0.01455927978213464300, 0.01304264340340427500, -0.00704666450967672550);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190321_222438 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190321_222438()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.22221431798593744000, 0.20154601678536010000, -0.50000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.19164624264091773000, 0.23080666732488140000, 0.50000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.22233983991481920000, 0.20140753607214537000, -0.50000000000000000000));
         pointsBeforeIssue.add(new Point3D(0.01577966270187325400, -0.29958471630744965000, 0.50000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.24694684726893160000, 0.17034451744607154000, 0.50000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.26566620349137715000, -0.13936092824919830000, -0.50000000000000000000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-6);

         troublesomePoint.set(-0.22089658280712426000, 0.20298940786192580000, -0.50000000000000000000);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190323_122756 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190323_122756()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.27845244126420843000, 0.11164335158889922000, -0.50000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.27827933053495096000, 0.11207414597943410000, -0.50000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.27870324942520370000, 0.11101575906074180000, -0.50000000000000000000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-6);

         troublesomePoint.set(0.24205618014166072000, -0.17722529631863476000, 0.50000000000000000000);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190323_124929 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190323_124929()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.04855499760929303000, -0.01193365858241087500, 0.15000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.02595809833327553000, -0.04273379378103467400, 0.15000000000000000000));
         pointsBeforeIssue.add(new Point3D(0.02574336606832163000, 0.04286349383184235000, -0.15000000000000000000));
         pointsBeforeIssue.add(new Point3D(0.04852589338486361400, -0.01205145929756432600, 0.15000000000000000000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-6);
         troublesomePoint.set(0.04841411994552912600, -0.01249291758957528400, 0.15000000000000000000);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190323_150735 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190323_150735()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.00127134727090134300, -0.04998383414781995000, -0.15000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.03069378997379051300, -0.03947013120126202600, 0.15000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.02968907227182072000, -0.04023131849242088000, -0.15000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.02949883371674418000, -0.04037101447018488000, 0.15000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.02973125211791738700, -0.04020015730691651000, -0.15000000000000000000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-6);
         troublesomePoint.set(-0.02889868575683430700, -0.04080276904240376000, -0.15000000000000000000);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190323_190624 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190323_190624()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.02299062921644618000, -0.04440079918460806000, -0.001));
         pointsBeforeIssue.add(new Point3D(-0.01961949482806400000, -0.04598994914860822500, 0.001));
         pointsBeforeIssue.add(new Point3D(-0.01954504118681128400, -0.04602164018161294000, 0.001));
         pointsBeforeIssue.add(new Point3D(-0.01853556656231905000, -0.04643740703585678000, 0.001));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-6);
         troublesomePoint.set(-0.01764619662299450600, -0.04678260087620843000, -0.001);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190323_193234 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190323_193234()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.01540314387381502300, -0.04756829993601363600, -0.001));
         pointsBeforeIssue.add(new Point3D(-0.02299062921644618000, -0.04440079918460806000, -0.001));
         pointsBeforeIssue.add(new Point3D(-0.01961949482806400000, -0.04598994914860822500, 0.001));
         pointsBeforeIssue.add(new Point3D(-0.01954504118681128400, -0.04602164018161294000, 0.001));
         pointsBeforeIssue.add(new Point3D(-0.01853556656231905000, -0.04643740703585678000, 0.001));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-6);
         troublesomePoint.set(-0.01764619662299450600, -0.04678260087620843000, -0.001);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190323_195449 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190323_195449()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(-0.03541014613115714400, -0.03530044689476462000, 0.001));
         pointsBeforeIssue.add(new Point3D(-0.03734847754427659000, -0.03324291240437076600, 0.001));
         pointsBeforeIssue.add(new Point3D(-0.03534070522258475000, -0.03536996684152207000, -0.001));
         pointsBeforeIssue.add(new Point3D(-0.03543195159362232000, -0.03527856015014224600, -0.001));
         pointsBeforeIssue.add(new Point3D(0.01847795496318006800, 0.04646036138880852000, -0.001));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-6);
         troublesomePoint.set(-0.03581713708947079000, -0.03488742883495509600, -0.001);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190323_213507 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190323_213507()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.01569971687419740800, 0.04747124276938661400, -0.001));
         pointsBeforeIssue.add(new Point3D(0.00638643724321071700, 0.04959045693819055000, 0.001));
         pointsBeforeIssue.add(new Point3D(-0.04099078792044472000, 0.02863136925927790000, 0.001));
         pointsBeforeIssue.add(new Point3D(0.00562149156111026600, 0.04968298333059687000, 0.001));
         pointsBeforeIssue.add(new Point3D(0.00653865510674285100, 0.04957061618938245000, 0.001));
         pointsBeforeIssue.add(new Point3D(0.00542547626947362700, 0.04970477046772652600, 0.001));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-6);
         troublesomePoint.set(0.00609110437244231200, 0.04962759764006328000, 0.001);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190323_224417 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190323_224417()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(1.40710695119247480000, -0.07785366545459171000, -0.12474415262385924000));
         pointsBeforeIssue.add(new Point3D(1.39819234947942730000, -0.06519028263078697000, -0.10973121290941995000));
         pointsBeforeIssue.add(new Point3D(1.16656489270930890000, 0.58035642697146620000, -0.78675217949450650000));
         pointsBeforeIssue.add(new Point3D(1.16657693484637440000, 0.58034005222896120000, -0.78677176678599660000));
         pointsBeforeIssue.add(new Point3D(1.17877634842965380000, 0.56298293314650350000, -0.80734273721939860000));
         pointsBeforeIssue.add(new Point3D(-0.23727513673035344000, -0.33079240501605790000, 0.19266613381248243000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-6);
         troublesomePoint.set(1.17401661111695740000, 0.56994514610194910000, -0.79913666286820100000);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190324_182459 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190324_182459()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(0.14002669077756869000, 1.71724580178321700000, -0.51140456321487700000));
         pointsBeforeIssue.add(new Point3D(0.14464138985533160000, 1.71884373875992600000, -0.50821583595323080000));
         pointsBeforeIssue.add(new Point3D(0.44125051252842373000, 1.80742794525663130000, -0.31821074227963140000));
         pointsBeforeIssue.add(new Point3D(1.66194586390551140000, 0.88972308060624930000, -0.88487843614218600000));
         pointsBeforeIssue.add(new Point3D(0.26804772520095743000, 0.71749313283470420000, -1.52109919425220790000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-6);
         troublesomePoint.set(0.11682083115045750000, 1.70320404009874780000, -0.53374773515644060000);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190324_185429 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190324_185429()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         pointsBeforeIssue.add(new Point3D(1.02789006910026280000, -1.96520915722745930000, -0.57037689610167770000));
         pointsBeforeIssue.add(new Point3D(1.53130112740555410000, -1.54277251686120880000, -0.21290799720003972000));
         pointsBeforeIssue.add(new Point3D(1.00266118280722630000, -1.88594311008552570000, 0.88496609188020230000));
         pointsBeforeIssue.add(new Point3D(0.45799000579097760000, -2.35848721332737950000, 0.27109853934318723000));
         pointsBeforeIssue.add(new Point3D(0.97953540922620610000, -2.00310093530315260000, -0.56532794195239730000));
         pointsBeforeIssue.add(new Point3D(1.01941156390882640000, -1.97187873769681400000, -0.56986767382176460000));
         pointsBeforeIssue.add(new Point3D(1.19731650748988640000, -1.73495118479849130000, 0.84202190182865110000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-6);
         troublesomePoint.set(1.03130688008012330000, -1.96251826135452710000, -0.57053698796407590000);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190325_205801 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190325_205801()
      {
         List<Point3D> pointsBeforeIssue = new ArrayList<>();
         //         pointsBeforeIssue.add(new Point3D(0.56611646535440970000, 0.72192533495156590000, -0.45456850180753970000));
         //         pointsBeforeIssue.add(new Point3D(-0.03096002954815313400, -1.11933802818877000000, 0.02485964480614530300));
         //         pointsBeforeIssue.add(new Point3D(-0.21134160734909313000, 0.05672553880322484000, 0.16969871696300470000));
         //         pointsBeforeIssue.add(new Point3D(-0.10998317783660272000, -0.35793171879201036000, -1.29657831706906200000));
         //         pointsBeforeIssue.add(new Point3D(1.24221610854542200000, -0.35793171879201060000, 0.38744131345398260000));
         //         pointsBeforeIssue.add(new Point3D(-0.22550967979103054000, 0.33265538375863757000, -0.70458601445941060000));
         //         pointsBeforeIssue.add(new Point3D(0.63924495047817190000, 0.33265538375863757000, 0.37237341572594795000));
         //         pointsBeforeIssue.add(new Point3D(-0.50140755744098240000, -0.51815796833610920000, -0.48305123748254850000));
         //         pointsBeforeIssue.add(new Point3D(0.36334707282821954000, -0.51815796833610930000, 0.59390819270281000000));
         //         pointsBeforeIssue.add(new Point3D(0.16744739763153893000, 0.53602251915225170000, 0.00150172460745828800));
         //         pointsBeforeIssue.add(new Point3D(-0.21926385954483330000, 0.34508339048342620000, -0.21996509622406915000));
         //         pointsBeforeIssue.add(new Point3D(0.17959578553533130000, 0.23399037342099427000, 0.36171653256595276000));
         //         pointsBeforeIssue.add(new Point3D(-0.07596089144905505000, 0.33369143486870023000, 0.07029846964374753000));
         //         pointsBeforeIssue.add(new Point3D(-0.45689190222145170000, -0.09027354986084624000, -0.23569812694162817000));
         //         pointsBeforeIssue.add(new Point3D(-0.18969810371658902000, -0.56534210088694260000, 0.28827503965931040000));
         //         pointsBeforeIssue.add(new Point3D(-0.25561638788651997000, 0.19714681531797900000, -0.02709429744593933000));
         //         pointsBeforeIssue.add(new Point3D(0.03812612018627559000, -0.19931008021384378000, 0.47395539540984977000));
         //         pointsBeforeIssue.add(new Point3D(-0.03233551185514816000, 0.19024727245488670000, 0.25778413632140140000));
         //         pointsBeforeIssue.add(new Point3D(-0.16508525859062940000, 0.20187788011925890000, 0.10938939412651272000));
         //         pointsBeforeIssue.add(new Point3D(-0.12909506210843258000, 0.13816978480641645000, 0.20623062556016397000));
         //         pointsBeforeIssue.add(new Point3D(-0.24863048016239442000, 0.11937632711577828000, 0.06931669611513780000));
         //         pointsBeforeIssue.add(new Point3D(-0.18557213452920450000, 0.13144707011625290000, 0.14673334110619220000));
         //         pointsBeforeIssue.add(new Point3D(-0.16992224571894476000, 0.09491743081785042000, 0.19293896534543290000));
         //         pointsBeforeIssue.add(new Point3D(-0.22788440734615910000, 0.09305892623498413000, 0.12185261922171944000));
         //         pointsBeforeIssue.add(new Point3D(-0.19670714571830983000, 0.09448477990656790000, 0.16147590131211610000));
         //         pointsBeforeIssue.add(new Point3D(-0.17622443579785530000, 0.11516757576961434000, 0.17088997042022797000));
         //         pointsBeforeIssue.add(new Point3D(-0.20659897343940292000, 0.11063137187004246000, 0.13657508247521946000));
         //         pointsBeforeIssue.add(new Point3D(-0.19007532622576317000, 0.11323054183299419000, 0.15557193696804306000));
         //         pointsBeforeIssue.add(new Point3D(-0.19585655386072087000, 0.12026034355340631000, 0.14277622782332633000));
         //         pointsBeforeIssue.add(new Point3D(-0.18009157945902032000, 0.12432472983314297000, 0.15918513617347450000));
         pointsBeforeIssue.add(new Point3D(-0.18876780767622203000, 0.12212619407332592000, 0.15023344192788570000));
         //         pointsBeforeIssue.add(new Point3D(-0.20711970910767297000, 0.15828615819577030000, 0.09471847100469560000));
         pointsBeforeIssue.add(new Point3D(-0.18540389512117350000, 0.11931502966653201000, 0.15661391400312075000));
         //         pointsBeforeIssue.add(new Point3D(-0.19359996445657135000, 0.11597170876463425000, 0.14903546118895833000));
         pointsBeforeIssue.add(new Point3D(-0.18910641566302666000, 0.11781600327121805000, 0.15320922191975280000));
         //         pointsBeforeIssue.add(new Point3D(-0.19106057386009068000, 0.11889604895538014000, 0.14992257977362047000));
         //         pointsBeforeIssue.add(new Point3D(-0.18689640965574605000, 0.12100304735476214000, 0.15344823839344585000));
         pointsBeforeIssue.add(new Point3D(-0.18898329704146322000, 0.11995027034057121000, 0.15168590833076218000));
         pointsBeforeIssue.add(new Point3D(-0.19010895477142298000, 0.11831685929715507000, 0.15156816788281408000));
         //         pointsBeforeIssue.add(new Point3D(-0.18836044620231740000, 0.11938529901374689000, 0.15290521034846627000));
         pointsBeforeIssue.add(new Point3D(-0.18987562790174484000, 0.12025818288072876000, 0.15032953041950530000));
         pointsBeforeIssue.add(new Point3D(-0.18942754382228977000, 0.11906857959442607000, 0.15182676701982100000));
         pointsBeforeIssue.add(new Point3D(-0.18893434992590175000, 0.12072587041103894000, 0.15113475488060024000));
         convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), 1.0e-6);
         troublesomePoint.set(-0.18974063099743910000, 0.11980345470548110000, 0.15085730736946484000);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190327_205357 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190327_205357()
      {
         double constructionEpsilon = 1.0E-6;
         Vertex3D v0 = new Vertex3D(0.56611646535440970000, 0.72192533495156590000, -0.45456850180753970000);
         Vertex3D v1 = new Vertex3D(-0.10998317783660272000, -0.35793171879201036000, -1.29657831706906200000);
         Vertex3D v2 = new Vertex3D(1.24221610854542200000, -0.35793171879201060000, 0.38744131345398260000);
         Vertex3D v3 = new Vertex3D(-0.03096002954815313400, -1.11933802818877000000, 0.02485964480614530300);
         Vertex3D v4 = new Vertex3D(-0.22550967979103054000, 0.33265538375863757000, -0.70458601445941060000);
         Vertex3D v5 = new Vertex3D(0.63924495047817190000, 0.33265538375863757000, 0.37237341572594795000);
         Vertex3D v6 = new Vertex3D(-0.50140755744098240000, -0.51815796833610920000, -0.48305123748254850000);
         Vertex3D v7 = new Vertex3D(0.36334707282821954000, -0.51815796833610930000, 0.59390819270281000000);
         Vertex3D v8 = new Vertex3D(0.16744739763153893000, 0.53602251915225170000, 0.00150172460745828800);
         Vertex3D v9 = new Vertex3D(-0.21926385954483330000, 0.34508339048342620000, -0.21996509622406915000);
         Vertex3D v10 = new Vertex3D(0.17959578553533130000, 0.23399037342099427000, 0.36171653256595276000);
         Vertex3D v11 = new Vertex3D(-0.07596089144905505000, 0.33369143486870023000, 0.07029846964374753000);
         Vertex3D v12 = new Vertex3D(-0.45689190222145170000, -0.09027354986084624000, -0.23569812694162817000);
         Vertex3D v13 = new Vertex3D(-0.21134160734909313000, 0.05672553880322484000, 0.16969871696300470000);
         Vertex3D v14 = new Vertex3D(-0.18969810371658902000, -0.56534210088694260000, 0.28827503965931040000);
         Vertex3D v15 = new Vertex3D(-0.25561638788651997000, 0.19714681531797900000, -0.02709429744593933000);
         Vertex3D v16 = new Vertex3D(0.03812612018627559000, -0.19931008021384378000, 0.47395539540984977000);
         Vertex3D v17 = new Vertex3D(-0.03233551185514816000, 0.19024727245488670000, 0.25778413632140140000);
         Vertex3D v18 = new Vertex3D(-0.16508525859062940000, 0.20187788011925890000, 0.10938939412651272000);
         Vertex3D v19 = new Vertex3D(-0.12909506210843258000, 0.13816978480641645000, 0.20623062556016397000);
         Vertex3D v20 = new Vertex3D(-0.24863048016239442000, 0.11937632711577828000, 0.06931669611513780000);
         Vertex3D v21 = new Vertex3D(-0.18557213452920450000, 0.13144707011625290000, 0.14673334110619220000);
         Vertex3D v22 = new Vertex3D(-0.16992224571894476000, 0.09491743081785042000, 0.19293896534543290000);
         Vertex3D v23 = new Vertex3D(-0.22788440734615910000, 0.09305892623498413000, 0.12185261922171944000);
         Vertex3D v24 = new Vertex3D(-0.19670714571830983000, 0.09448477990656790000, 0.16147590131211610000);
         Vertex3D v25 = new Vertex3D(-0.17622443579785530000, 0.11516757576961434000, 0.17088997042022797000);
         Vertex3D v26 = new Vertex3D(-0.20659897343940292000, 0.11063137187004246000, 0.13657508247521946000);
         Vertex3D v27 = new Vertex3D(-0.19007532622576317000, 0.11323054183299419000, 0.15557193696804306000);
         Vertex3D v28 = new Vertex3D(-0.18009157945902032000, 0.12432472983314297000, 0.15918513617347450000);
         Vertex3D v29 = new Vertex3D(-0.19585655386072087000, 0.12026034355340631000, 0.14277622782332633000);
         Vertex3D v30 = new Vertex3D(-0.18876780767622203000, 0.12212619407332592000, 0.15023344192788570000);
         Vertex3D v31 = new Vertex3D(-0.20711970910767297000, 0.15828615819577030000, 0.09471847100469560000);
         Vertex3D v32 = new Vertex3D(-0.18540389512117350000, 0.11931502966653201000, 0.15661391400312075000);
         Vertex3D v33 = new Vertex3D(-0.19359996445657135000, 0.11597170876463425000, 0.14903546118895833000);
         Vertex3D v34 = new Vertex3D(-0.19010895477142298000, 0.11831685929715507000, 0.15156816788281408000);
         Vertex3D v35 = new Vertex3D(-0.18910641566302666000, 0.11781600327121805000, 0.15320922191975280000);
         Vertex3D v36 = new Vertex3D(-0.18987562790174484000, 0.12025818288072876000, 0.15032953041950530000);
         Vertex3D v37 = new Vertex3D(-0.19106057386009068000, 0.11889604895538014000, 0.14992257977362047000);
         Vertex3D v38 = new Vertex3D(-0.18689640965574605000, 0.12100304735476214000, 0.15344823839344585000);
         Vertex3D v39 = new Vertex3D(-0.18836044620231740000, 0.11938529901374689000, 0.15290521034846627000);
         Vertex3D v40 = new Vertex3D(-0.18898329704146322000, 0.11995027034057121000, 0.15168590833076218000);
         Vertex3D v41 = new Vertex3D(-0.18893434992590175000, 0.12072587041103894000, 0.15113475488060024000);
         Vertex3D v42 = new Vertex3D(-0.18942754382228977000, 0.11906857959442607000, 0.15182676701982100000);
         List<Face3D> faces = new ArrayList<>();
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v1), new HalfEdge3D(v1, v2), new HalfEdge3D(v2, v0)),
                              new Vector3D(0.77974192263920370000, -0.00000000000000009521, -0.62610105740121360000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v1, v3), new HalfEdge3D(v3, v2), new HalfEdge3D(v2, v1)),
                              new Vector3D(0.54979538036117710000, -0.70910871118184650000, -0.44146333421874234000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v1, v0), new HalfEdge3D(v0, v4), new HalfEdge3D(v4, v1)),
                              new Vector3D(-0.07454392332283849000, 0.64180156697008430000, -0.76323911857974300000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v2), new HalfEdge3D(v2, v5), new HalfEdge3D(v5, v0)),
                              new Vector3D(0.72912135612061290000, 0.64180156697008430000, 0.23764005698447144000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v1, v4), new HalfEdge3D(v4, v6), new HalfEdge3D(v6, v1)),
                              new Vector3D(-0.89895151216755550000, 0.18855645528825300000, -0.39538922840764710000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v3, v1), new HalfEdge3D(v1, v6), new HalfEdge3D(v6, v3)),
                              new Vector3D(-0.52592770943899230000, -0.75017489699518560000, -0.40079629285026770000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v2, v3), new HalfEdge3D(v3, v7), new HalfEdge3D(v7, v2)),
                              new Vector3D(0.27773757000445903000, -0.75017489699518560000, 0.60008288271394690000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v5, v2), new HalfEdge3D(v2, v7), new HalfEdge3D(v7, v5)),
                              new Vector3D(0.19188689125933103000, 0.18855645528825307000, 0.96313336778037050000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v5), new HalfEdge3D(v5, v8), new HalfEdge3D(v8, v0)),
                              new Vector3D(0.05935264926148290000, 0.90518507577347680000, 0.42085299288778740000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v4, v0), new HalfEdge3D(v0, v9), new HalfEdge3D(v9, v4)),
                              new Vector3D(-0.43676301882924334000, 0.89940760930531040000, -0.01743610354614344000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v8), new HalfEdge3D(v8, v9), new HalfEdge3D(v9, v0)),
                              new Vector3D(-0.43604452678201550000, 0.89981012662538550000, -0.01438425131370123000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v5, v7), new HalfEdge3D(v7, v10), new HalfEdge3D(v10, v5)),
                              new Vector3D(-0.08140272784372796000, 0.27578042616710907000, 0.95776758790569220000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v8, v5), new HalfEdge3D(v5, v10), new HalfEdge3D(v10, v8)),
                              new Vector3D(-0.17603085564385015000, 0.75138234096607160000, 0.63595417723735430000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v8, v10), new HalfEdge3D(v10, v11), new HalfEdge3D(v11, v8)),
                              new Vector3D(-0.40783426164494840000, 0.69283716564324720000, 0.59468300541713100000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v9, v8), new HalfEdge3D(v8, v11), new HalfEdge3D(v11, v9)),
                              new Vector3D(-0.55632270745373100000, 0.77297110680139450000, 0.30499297241342743000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v4, v9), new HalfEdge3D(v9, v12), new HalfEdge3D(v12, v4)),
                              new Vector3D(-0.87774416956391610000, 0.47912860765248216000, -0.00097474383569708630),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v6, v4), new HalfEdge3D(v4, v12), new HalfEdge3D(v12, v6)),
                              new Vector3D(-0.93886549465982120000, 0.24023839562892355000, -0.24661122481105790000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v12, v13), new HalfEdge3D(v13, v14), new HalfEdge3D(v14, v12)),
                              new Vector3D(-0.86381177330430940000, 0.06517396091166652000, 0.49958139989390005000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v6, v12), new HalfEdge3D(v12, v14), new HalfEdge3D(v14, v6)),
                              new Vector3D(-0.92337707251808760000, -0.11556401088414471000, 0.36608706797194523000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v3, v6), new HalfEdge3D(v6, v14), new HalfEdge3D(v14, v3)),
                              new Vector3D(-0.85828364091065290000, -0.39923562533379914000, 0.32242845285347240000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v7, v3), new HalfEdge3D(v3, v14), new HalfEdge3D(v14, v7)),
                              new Vector3D(-0.39119566121186483000, -0.48421494998826620000, 0.78262496564885590000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v9, v11), new HalfEdge3D(v11, v15), new HalfEdge3D(v15, v9)),
                              new Vector3D(-0.68040773809443640000, 0.63777895053491530000, 0.36094780813269270000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v12, v9), new HalfEdge3D(v9, v15), new HalfEdge3D(v15, v12)),
                              new Vector3D(-0.86408904005581990000, 0.46464624680883765000, 0.19353034951105855000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v14, v13), new HalfEdge3D(v13, v16), new HalfEdge3D(v16, v14)),
                              new Vector3D(-0.72400262863476380000, 0.10477105963985398000, 0.68179411759848340000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v7, v14), new HalfEdge3D(v14, v16), new HalfEdge3D(v16, v7)),
                              new Vector3D(-0.46836011998279030000, -0.15016447478870562000, 0.87068331126830390000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v10, v7), new HalfEdge3D(v7, v16), new HalfEdge3D(v16, v10)),
                              new Vector3D(-0.08334517953812422000, 0.27529722201467943000, 0.95773953692993090000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v10, v16), new HalfEdge3D(v16, v17), new HalfEdge3D(v17, v10)),
                              new Vector3D(-0.46961519763372760000, 0.36191976922094204000, 0.80527985619813880000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v11, v10), new HalfEdge3D(v10, v17), new HalfEdge3D(v17, v11)),
                              new Vector3D(-0.43541679761518950000, 0.66321358945884230000, 0.60873635271080320000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v11, v17), new HalfEdge3D(v17, v18), new HalfEdge3D(v18, v11)),
                              new Vector3D(-0.59027046098820020000, 0.56892553373242140000, 0.57262947876620630000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v15, v11), new HalfEdge3D(v11, v18), new HalfEdge3D(v18, v15)),
                              new Vector3D(-0.68237631262196360000, 0.58955332858006870000, 0.43219143991131220000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v17, v16), new HalfEdge3D(v16, v19), new HalfEdge3D(v19, v17)),
                              new Vector3D(-0.57297770148089210000, 0.31599154140635877000, 0.75620493212178090000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v18, v17), new HalfEdge3D(v17, v19), new HalfEdge3D(v19, v18)),
                              new Vector3D(-0.60344663802777170000, 0.54455351498763290000, 0.58250632989487180000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v12, v15), new HalfEdge3D(v15, v20), new HalfEdge3D(v20, v12)),
                              new Vector3D(-0.86760220789455310000, 0.35443459563009800000, 0.34877288638457205000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v18, v19), new HalfEdge3D(v19, v21), new HalfEdge3D(v21, v18)),
                              new Vector3D(-0.65810405783623990000, 0.49311416277672654000, 0.56897932434182800000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v16, v13), new HalfEdge3D(v13, v22), new HalfEdge3D(v22, v16)),
                              new Vector3D(-0.63720615128903680000, 0.24676755083455865000, 0.73011923451894270000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v19, v16), new HalfEdge3D(v16, v22), new HalfEdge3D(v22, v19)),
                              new Vector3D(-0.57694180564348130000, 0.31271529855622304000, 0.75455105523067930000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v13, v12), new HalfEdge3D(v12, v23), new HalfEdge3D(v23, v13)),
                              new Vector3D(-0.86846849503871360000, 0.20121576656995235000, 0.45307249796126280000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v12, v20), new HalfEdge3D(v20, v23), new HalfEdge3D(v23, v12)),
                              new Vector3D(-0.86867054800199670000, 0.21046189859871320000, 0.44846099972253520000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v13, v23), new HalfEdge3D(v23, v24), new HalfEdge3D(v24, v13)),
                              new Vector3D(-0.72630448391905990000, 0.40279199330498633000, 0.55699228609421840000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v22, v13), new HalfEdge3D(v13, v24), new HalfEdge3D(v24, v22)),
                              new Vector3D(-0.70052659988671840000, 0.40017745323459910000, 0.59086418809555850000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v22, v24), new HalfEdge3D(v24, v25), new HalfEdge3D(v25, v22)),
                              new Vector3D(-0.69360761582348750000, 0.42076727405888470000, 0.58469083826644030000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v19, v22), new HalfEdge3D(v22, v25), new HalfEdge3D(v25, v19)),
                              new Vector3D(-0.66607567833391500000, 0.44471088828418565000, 0.59881167037183360000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v24, v23), new HalfEdge3D(v23, v26), new HalfEdge3D(v26, v24)),
                              new Vector3D(-0.72332337926704910000, 0.41173701913067150000, 0.55432473883373220000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v25, v24), new HalfEdge3D(v24, v27), new HalfEdge3D(v27, v25)),
                              new Vector3D(-0.69667679566416640000, 0.42783137533501725000, 0.57584872723837530000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v19, v25), new HalfEdge3D(v25, v28), new HalfEdge3D(v28, v19)),
                              new Vector3D(-0.66507095264885750000, 0.46552084667686080000, 0.58392719516391720000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v21, v19), new HalfEdge3D(v19, v28), new HalfEdge3D(v28, v21)),
                              new Vector3D(-0.65922950841118770000, 0.48995398977437990000, 0.57040471872531910000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v29, v21), new HalfEdge3D(v21, v30), new HalfEdge3D(v30, v29)),
                              new Vector3D(-0.70114434300667240000, 0.44852535088474504000, 0.55427576158754970000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v23, v20), new HalfEdge3D(v20, v31), new HalfEdge3D(v31, v23)),
                              new Vector3D(-0.73282019574173680000, 0.44677430385297495000, 0.51319322104810800000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v26, v23), new HalfEdge3D(v23, v31), new HalfEdge3D(v31, v26)),
                              new Vector3D(-0.72836410096192520000, 0.44766087198529586000, 0.51873449868240970000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v29, v26), new HalfEdge3D(v26, v31), new HalfEdge3D(v31, v29)),
                              new Vector3D(-0.71501163875767040000, 0.45693177292667970000, 0.52912353125820440000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v21, v29), new HalfEdge3D(v29, v31), new HalfEdge3D(v31, v21)),
                              new Vector3D(-0.70856159221403430000, 0.46306077685711730000, 0.53246144177357660000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v18, v21), new HalfEdge3D(v21, v31), new HalfEdge3D(v31, v18)),
                              new Vector3D(-0.69047474628578290000, 0.48527136661633985000, 0.53642923623148460000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v15, v18), new HalfEdge3D(v18, v31), new HalfEdge3D(v31, v15)),
                              new Vector3D(-0.71276846310571180000, 0.53442230722506070000, 0.45426194595427494000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v20, v15), new HalfEdge3D(v15, v31), new HalfEdge3D(v31, v20)),
                              new Vector3D(-0.74133949575435320000, 0.49509937789431450000, 0.45309199732873650000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v25, v27), new HalfEdge3D(v27, v32), new HalfEdge3D(v32, v25)),
                              new Vector3D(-0.69452811078689790000, 0.43510681876864320000, 0.57298582843541010000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v28, v25), new HalfEdge3D(v25, v32), new HalfEdge3D(v32, v28)),
                              new Vector3D(-0.69187737803803810000, 0.43976659756973124000, 0.57263516607099130000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v24, v26), new HalfEdge3D(v26, v33), new HalfEdge3D(v33, v24)),
                              new Vector3D(-0.71106398826683910000, 0.42648348767489920000, 0.55901685066794790000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v27, v24), new HalfEdge3D(v24, v33), new HalfEdge3D(v33, v27)),
                              new Vector3D(-0.70860113346811680000, 0.42748999606631740000, 0.56137040972153140000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v26, v29), new HalfEdge3D(v29, v33), new HalfEdge3D(v33, v26)),
                              new Vector3D(-0.70978355893156320000, 0.43508079539005884000, 0.55399639074022660000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v27, v33), new HalfEdge3D(v33, v34), new HalfEdge3D(v34, v35), new HalfEdge3D(v35, v27)),
                              new Vector3D(-0.70175342024421780000, 0.43780435108074900000, 0.56202267512112560000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v32, v27), new HalfEdge3D(v27, v35), new HalfEdge3D(v35, v32)),
                              new Vector3D(-0.69799007937168470000, 0.43900177475399390000, 0.56576257463847220000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v29, v30), new HalfEdge3D(v30, v36), new HalfEdge3D(v36, v37), new HalfEdge3D(v37, v29)),
                              new Vector3D(-0.70207503239470800000, 0.44482862644756510000, 0.55607386378137050000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v33, v29), new HalfEdge3D(v29, v37), new HalfEdge3D(v37, v33)),
                              new Vector3D(-0.70348169396268320000, 0.44205825363331763000, 0.55650517217188090000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v21, v28), new HalfEdge3D(v28, v38), new HalfEdge3D(v38, v21)),
                              new Vector3D(-0.69377446233767970000, 0.44966481132332026000, 0.56256426554272580000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v30, v21), new HalfEdge3D(v21, v38), new HalfEdge3D(v38, v30)),
                              new Vector3D(-0.69499918040443040000, 0.44912483371224965000, 0.56148287861707250000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v32, v35), new HalfEdge3D(v35, v39), new HalfEdge3D(v39, v38), new HalfEdge3D(v38, v32)),
                              new Vector3D(-0.69759573778429120000, 0.44141468321204136000, 0.56436979372571260000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v28, v32), new HalfEdge3D(v32, v38), new HalfEdge3D(v38, v28)),
                              new Vector3D(-0.69410024963427760000, 0.44600740212414575000, 0.56506835047461620000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v30, v38),
                                            new HalfEdge3D(v38, v39),
                                            new HalfEdge3D(v39, v40),
                                            new HalfEdge3D(v40, v41),
                                            new HalfEdge3D(v41, v30)),
                              new Vector3D(-0.69822959062416780000, 0.44380048177333326000, 0.56170861766093970000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v33, v37), new HalfEdge3D(v37, v34), new HalfEdge3D(v34, v33)),
                              new Vector3D(-0.70189294643401580000, 0.43944726247160830000, 0.56056435424702590000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v34, v37),
                                            new HalfEdge3D(v37, v36),
                                            new HalfEdge3D(v36, v41),
                                            new HalfEdge3D(v41, v42),
                                            new HalfEdge3D(v42, v34)),
                              new Vector3D(-0.70016999203383450000, 0.44211587774038110000, 0.56062066756871560000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v35, v34), new HalfEdge3D(v34, v42), new HalfEdge3D(v42, v39), new HalfEdge3D(v39, v35)),
                              new Vector3D(-0.69947759231787510000, 0.44126117424480665000, 0.56215627182242020000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v40, v39), new HalfEdge3D(v39, v42), new HalfEdge3D(v42, v40)),
                              new Vector3D(-0.69913829852196350000, 0.44204525633165100000, 0.56196230380204190000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v36, v30), new HalfEdge3D(v30, v41), new HalfEdge3D(v41, v36)),
                              new Vector3D(-0.69959231065829750000, 0.44370213829446964000, 0.56008839600611220000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v40, v42), new HalfEdge3D(v42, v41), new HalfEdge3D(v41, v40)),
                              new Vector3D(-0.70027522920976590000, 0.44235288603661520000, 0.56030217523251400000),
                              constructionEpsilon));
         convexPolytope3D = new ConvexPolytope3D(faces, constructionEpsilon);
         troublesomePoint.set(-0.18974063099743910000, 0.11980345470548110000, 0.15085730736946484000);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190327_211921 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190327_211921()
      {
         double constructionEpsilon = 1.0E-6;
         Vertex3D v0 = new Vertex3D(2.32394937602086900000, -1.23790293146439120000, -0.76787945767079640000);
         Vertex3D v1 = new Vertex3D(2.50062388613901300000, -0.79366196390220300000, -0.93074781967068440000);
         Vertex3D v2 = new Vertex3D(2.63217804755516930000, -0.54432263398845850000, -0.10877052699584278000);
         Vertex3D v3 = new Vertex3D(2.45520990991262800000, -0.98917066217772060000, 0.05290903670859981000);
         List<Face3D> faces = new ArrayList<>();
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v1), new HalfEdge3D(v1, v2), new HalfEdge3D(v2, v3), new HalfEdge3D(v3, v0)),
                              new Vector3D(-0.92452616874505020000, 0.37970580001758520000, 0.03278519084288889000),
                              constructionEpsilon));
         convexPolytope3D = new ConvexPolytope3D(faces, constructionEpsilon);
         troublesomePoint.set(2.63188242034863700000, -0.54493368729793970000, -0.10996947561500714000);
      }
   }

   /**
    * Dataset obtained from EPA. The silhouette is corrupted and it seems that there is no easy
    * solution to properly handle that scenario. The main problem here was about to isolate this type
    * of situation. It seems that if an {@code inPlaneFace} has an edge that is part of the silhouette
    * but is not visible by the new vertex, then we should abort and throw away the new vertex.
    *
    * @author Sylvain Bertrand
    */
   public static class ConvexPolytope3DTroublesomeDataset_20190327_213133 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190327_213133()
      {
         double constructionEpsilon = 1.0E-6;
         Vertex3D v0 = new Vertex3D(-1.82205152808490970000, -0.04140574974687161000, -0.49019311771537233000);
         Vertex3D v1 = new Vertex3D(-0.02898375989394031000, -1.62716384827626870000, 0.03663767132663342500);
         Vertex3D v2 = new Vertex3D(-0.54086231054945980000, -0.66608011256551690000, -1.02965626657008570000);
         Vertex3D v3 = new Vertex3D(0.89636856046639270000, -0.04140574974687161000, 1.66032661469346990000);
         Vertex3D v4 = new Vertex3D(1.12649293489673320000, -0.66608011256551690000, 0.28937472917947216000);
         Vertex3D v5 = new Vertex3D(-0.46284148380925850000, 1.69169428323730870000, 0.58506674848904880000);
         Vertex3D v6 = new Vertex3D(-0.76148415814494970000, 1.02159898710346430000, -0.75077351397841460000);
         Vertex3D v7 = new Vertex3D(0.90587108730124380000, 1.02159898710346430000, 0.56825748177114390000);
         Vertex3D v8 = new Vertex3D(-1.44668662165818280000, -0.74069608807437830000, 1.82872164104670660000);
         Vertex3D v9 = new Vertex3D(0.50400346350016480000, -0.79979773288111140000, -0.63709861352614140000);
         Vertex3D v10 = new Vertex3D(-0.29759310416984786000, 0.26086992828859610000, -1.11344969588954920000);
         Vertex3D v11 = new Vertex3D(1.15205016461202050000, 0.26086992828859656000, 0.03335114905654240600);
         Vertex3D v12 = new Vertex3D(0.23195279387966772000, 1.28129366479082970000, -0.29320592830450410000);
         Vertex3D v13 = new Vertex3D(0.92489277312831320000, -0.24590133679278380000, -0.43277946353437920000);
         Vertex3D v14 = new Vertex3D(0.20830385932479856000, -0.24590133679278360000, -0.99966704858791920000);
         Vertex3D v15 = new Vertex3D(0.06959193450038414000, 0.81519582539870640000, -0.82432465238403430000);
         Vertex3D v16 = new Vertex3D(0.78618084830389980000, 0.81519582539870640000, -0.25743706733049404000);
         Vertex3D v17 = new Vertex3D(0.58931652978928220000, 0.31591645181884440000, -0.74494080149642790000);
         Vertex3D v18 = new Vertex3D(0.93209716265175800000, 0.24255515001659209000, -0.40344127087386900000);
         Vertex3D v19 = new Vertex3D(0.17809515696868494000, 0.24255515001659280000, -0.99992604463134150000);
         Vertex3D v20 = new Vertex3D(0.77037617918571270000, 0.08108457521372880000, -0.62646370523385930000);
         Vertex3D v21 = new Vertex3D(0.43235000074393340000, 0.08108457521372836000, -0.89387342968053030000);
         Vertex3D v22 = new Vertex3D(0.70264301906135840000, -0.04572758614330080000, -0.69761127799630470000);
         Vertex3D v23 = new Vertex3D(0.61014181632540590000, -0.15412343022580083000, -0.77126554356527740000);
         Vertex3D v24 = new Vertex3D(0.51717610034563300000, -0.04572758614330119000, -0.84433262915428180000);
         Vertex3D v25 = new Vertex3D(0.67732544864221390000, 0.18955638130512897000, -0.69957778775196360000);
         Vertex3D v26 = new Vertex3D(0.52491650442851680000, 0.18955638130512895000, -0.82014724767742450000);
         Vertex3D v27 = new Vertex3D(0.60498147214801070000, -0.04573761550240424000, -0.77998676474752870000);
         Vertex3D v28 = new Vertex3D(0.52302044103363070000, 0.06269195753014367000, -0.83650831348077550000);
         Vertex3D v29 = new Vertex3D(0.69368369556123400000, 0.06269195753014306000, -0.70149801908057790000);
         Vertex3D v30 = new Vertex3D(0.60490312079108680000, 0.16314156583298078000, -0.76464343498209030000);
         Vertex3D v31 = new Vertex3D(0.65420016888614320000, 0.00330838000271832520, -0.73926490071141870000);
         Vertex3D v32 = new Vertex3D(0.56860702737008870000, 0.01280369397397235200, -0.80662683464164580000);
         Vertex3D v33 = new Vertex3D(0.56503390882937600000, 0.11289290517597522000, -0.80176410876310400000);
         Vertex3D v34 = new Vertex3D(0.65020300288733250000, 0.11289290517597472000, -0.73438754024288410000);
         Vertex3D v35 = new Vertex3D(0.61193961305573910000, 0.00808052836364181500, -0.77362101050129750000);
         Vertex3D v36 = new Vertex3D(0.65281471672976020000, 0.05791057734361360000, -0.73746235656001000000);
         Vertex3D v37 = new Vertex3D(0.70298527602064680000, -0.48320460347542280000, -0.61880096153494310000);
         Vertex3D v38 = new Vertex3D(0.57056392569500540000, 0.06289063269897044000, -0.80231612892648400000);
         Vertex3D v39 = new Vertex3D(0.60522083226107080000, 0.11296833365890513000, -0.77104801645264940000);
         Vertex3D v40 = new Vertex3D(0.63090292911791730000, 0.03535070061411177400, -0.75712076853245040000);
         Vertex3D v41 = new Vertex3D(0.59155957440437620000, 0.03539785374539334000, -0.78824142495868010000);
         Vertex3D v42 = new Vertex3D(0.63243635314749040000, 0.08539842333997033000, -0.75206558822593580000);
         Vertex3D v43 = new Vertex3D(0.61128552250191000000, 0.03752602977256564000, -0.77274644662457560000);
         Vertex3D v44 = new Vertex3D(0.58976365987554720000, 0.08611143241391384000, -0.78584444632695040000);
         Vertex3D v45 = new Vertex3D(0.63017540416641050000, 0.06056589168600967600, -0.75603413931665090000);
         Vertex3D v46 = new Vertex3D(0.59070310644506030000, 0.06061400763941965400, -0.78725664261760630000);
         Vertex3D v47 = new Vertex3D(0.62000841854423240000, 0.05008581903291520000, -0.76500773321372630000);
         Vertex3D v48 = new Vertex3D(0.60176136376824540000, 0.05010594922548295000, -0.77944126182486300000);
         Vertex3D v49 = new Vertex3D(0.61227552221037070000, 0.08823698491407433000, -0.76805996732391870000);
         Vertex3D v50 = new Vertex3D(0.61047800660565690000, 0.06266503211187174000, -0.77169051358421680000);
         Vertex3D v51 = new Vertex3D(0.62066889447759950000, 0.07377384915251545000, -0.76266269189274150000);
         Vertex3D v52 = new Vertex3D(0.59941412048978240000, 0.07331142171150717000, -0.77951697046616260000);
         Vertex3D v53 = new Vertex3D(0.61981680212005870000, 0.06199059277688501000, -0.76430197672695770000);
         Vertex3D v54 = new Vertex3D(0.60133722271800690000, 0.06178625892945938000, -0.77893868194334770000);
         Vertex3D v55 = new Vertex3D(0.61510936938194770000, 0.05683445549448912000, -0.76844961436300540000);
         Vertex3D v56 = new Vertex3D(0.60642747567082580000, 0.05684528871167899000, -0.77531788464378750000);
         Vertex3D v57 = new Vertex3D(0.60881959381868080000, 0.05611856820290173000, -0.77348662419658950000);
         Vertex3D v58 = new Vertex3D(0.61077141492718110000, 0.05684037625856802600, -0.77189263436713330000);
         Vertex3D v59 = new Vertex3D(0.61275346975400690000, 0.05609096794727802500, -0.77037620871640760000);
         Vertex3D v60 = new Vertex3D(0.61525727422694690000, 0.04426657468532402000, -0.76918546032470980000);
         Vertex3D v61 = new Vertex3D(0.60691296723952050000, 0.04427477209551056000, -0.77578592060041230000);
         Vertex3D v62 = new Vertex3D(0.61087947580614020000, 0.05101482260928985600, -0.77221636973563250000);
         Vertex3D v63 = new Vertex3D(0.61543889344682860000, 0.05054155603611365500, -0.76862924593270040000);
         Vertex3D v64 = new Vertex3D(0.60634183931468970000, 0.05055622841335677400, -0.77582477564366340000);
         Vertex3D v65 = new Vertex3D(0.60850870633951510000, 0.05374515059351154000, -0.77390006514635750000);
         Vertex3D v66 = new Vertex3D(0.61315351911568760000, 0.05373784690034522000, -0.77022590405037370000);
         Vertex3D v67 = new Vertex3D(0.61179518432069820000, 0.05355605372880795000, -0.77131594519329070000);
         Vertex3D v68 = new Vertex3D(0.61082657652464620000, 0.05392616471100958000, -0.77205684449516940000);
         Vertex3D v69 = new Vertex3D(0.61079926615714570000, 0.05573606365719769000, -0.77195010197004300000);
         List<Face3D> faces = new ArrayList<>();
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v1), new HalfEdge3D(v1, v2), new HalfEdge3D(v2, v0)),
                              new Vector3D(-0.53343389697778020000, -0.73955087529481420000, -0.41050308209046754000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v1, v3), new HalfEdge3D(v3, v4), new HalfEdge3D(v4, v1)),
                              new Vector3D(0.52225030474281360000, -0.73955087529481420000, 0.42463999110624530000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v5, v0), new HalfEdge3D(v0, v6), new HalfEdge3D(v6, v5)),
                              new Vector3D(-0.71809487864477720000, 0.67304031945918660000, -0.17707759216348860000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v3, v5), new HalfEdge3D(v5, v7), new HalfEdge3D(v7, v3)),
                              new Vector3D(0.33758932307581650000, 0.67304031945918660000, 0.65806548103322370000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v5), new HalfEdge3D(v5, v8), new HalfEdge3D(v8, v0)),
                              new Vector3D(-0.83204526346120360000, 0.47930678004792265000, 0.27922336963776623000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v1, v0), new HalfEdge3D(v0, v8), new HalfEdge3D(v8, v1)),
                              new Vector3D(-0.63601387424549530000, -0.76122168658611360000, -0.12660132557847903000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v3, v1), new HalfEdge3D(v1, v8), new HalfEdge3D(v8, v3)),
                              new Vector3D(0.26957757461803783000, -0.76122168658611360000, 0.58980460758970780000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v5, v3), new HalfEdge3D(v3, v8), new HalfEdge3D(v8, v5)),
                              new Vector3D(-0.08023800787100821000, 0.47930678004792270000, 0.87397189468139320000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v2, v1), new HalfEdge3D(v1, v9), new HalfEdge3D(v9, v2)),
                              new Vector3D(0.17667773796444133000, -0.68743831064915520000, -0.70442426559538950000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v1, v4), new HalfEdge3D(v4, v9), new HalfEdge3D(v9, v1)),
                              new Vector3D(0.64485385726953810000, -0.68743831064915520000, -0.33405399536068090000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v2), new HalfEdge3D(v2, v10), new HalfEdge3D(v10, v0)),
                              new Vector3D(-0.38117859819855304000, 0.01647788529582807000, -0.92435456161127030000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v6, v0), new HalfEdge3D(v0, v10), new HalfEdge3D(v10, v6)),
                              new Vector3D(-0.40281734336076440000, 0.18200663211461800000, -0.89700154612691160000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v3, v7), new HalfEdge3D(v7, v11), new HalfEdge3D(v11, v3)),
                              new Vector3D(0.96562891965509190000, 0.18200663211461793000, 0.18556501661691013000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v4, v3), new HalfEdge3D(v3, v11), new HalfEdge3D(v11, v4)),
                              new Vector3D(0.98726766481730310000, 0.01647788529582871700, 0.15821200113255166000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v5, v6), new HalfEdge3D(v6, v12), new HalfEdge3D(v12, v5)),
                              new Vector3D(-0.03033480671910025400, 0.89613443538402670000, -0.44274470433904284000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v7, v5), new HalfEdge3D(v5, v12), new HalfEdge3D(v12, v7)),
                              new Vector3D(0.43784131258599750000, 0.89613443538402680000, -0.07237443410433325000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v4, v11), new HalfEdge3D(v11, v13), new HalfEdge3D(v13, v4)),
                              new Vector3D(0.93688567003073890000, -0.11684807863016765000, -0.32953265060004533000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v2, v9), new HalfEdge3D(v9, v14), new HalfEdge3D(v14, v2)),
                              new Vector3D(0.27112452691889900000, -0.42164997122471370000, -0.86527613665765770000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v10, v2), new HalfEdge3D(v2, v14), new HalfEdge3D(v14, v10)),
                              new Vector3D(0.10506844777430578000, -0.11684807863016812000, -0.98757640099525260000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v6, v10), new HalfEdge3D(v10, v15), new HalfEdge3D(v15, v6)),
                              new Vector3D(0.03152554391539832000, 0.44572912360236650000, -0.89461259126702710000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v12, v6), new HalfEdge3D(v6, v15), new HalfEdge3D(v15, v12)),
                              new Vector3D(0.12090366033912281000, 0.72748652465708990000, -0.67538556496193450000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v12, v15), new HalfEdge3D(v15, v16), new HalfEdge3D(v16, v12)),
                              new Vector3D(0.51363065513530670000, 0.56092287001550810000, -0.64926811410914140000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v7, v12), new HalfEdge3D(v12, v16), new HalfEdge3D(v16, v7)),
                              new Vector3D(0.62943071184101750000, 0.72748652465709000000, -0.27309400475597995000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v11, v7), new HalfEdge3D(v7, v16), new HalfEdge3D(v16, v11)),
                              new Vector3D(0.86334276617183180000, 0.44572912360236660000, -0.23656884087181926000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v15, v17), new HalfEdge3D(v17, v16), new HalfEdge3D(v16, v15)),
                              new Vector3D(0.55019605343337610000, 0.46214563339872740000, -0.69548955155115510000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v16, v17), new HalfEdge3D(v17, v18), new HalfEdge3D(v18, v16)),
                              new Vector3D(0.69945784819127650000, 0.33868767594496590000, -0.62932454009567050000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v11, v16), new HalfEdge3D(v16, v18), new HalfEdge3D(v18, v11)),
                              new Vector3D(0.83911196281210610000, 0.32502673046230780000, -0.43617512348894555000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v13, v11), new HalfEdge3D(v11, v18), new HalfEdge3D(v18, v13)),
                              new Vector3D(0.89283056974527410000, 0.01387051019632242000, -0.45017905623787400000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v10, v14), new HalfEdge3D(v14, v19), new HalfEdge3D(v19, v10)),
                              new Vector3D(0.23261530592369570000, 0.01387051019632302300, -0.97246991130673000000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v15, v10), new HalfEdge3D(v10, v19), new HalfEdge3D(v19, v15)),
                              new Vector3D(0.23135029069674468000, 0.32502673046230790000, -0.91697037437395650000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v17, v15), new HalfEdge3D(v15, v19), new HalfEdge3D(v19, v17)),
                              new Vector3D(0.45145545593760117000, 0.33868767594496600000, -0.82551718908037890000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v13, v18), new HalfEdge3D(v18, v20), new HalfEdge3D(v20, v13)),
                              new Vector3D(0.80094365333947190000, 0.02411959518940798300, -0.59825371649753980000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v19, v14), new HalfEdge3D(v14, v21), new HalfEdge3D(v21, v19)),
                              new Vector3D(0.39786234824481465000, 0.02411959518940872500, -0.91712801558834780000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v13, v20), new HalfEdge3D(v20, v22), new HalfEdge3D(v22, v13)),
                              new Vector3D(0.75297529613603680000, -0.03346203119162343600, -0.65719745577518710000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v14, v23), new HalfEdge3D(v23, v24), new HalfEdge3D(v24, v14)),
                              new Vector3D(0.51247869941988910000, -0.13238205284815271000, -0.84843418997857920000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v21, v14), new HalfEdge3D(v14, v24), new HalfEdge3D(v24, v21)),
                              new Vector3D(0.46626345935726540000, -0.03346203119162337000, -0.88401282735983330000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v18, v17), new HalfEdge3D(v17, v25), new HalfEdge3D(v25, v18)),
                              new Vector3D(0.70901843509578020000, 0.25825185437921705000, -0.65620030356898410000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v20, v18), new HalfEdge3D(v18, v25), new HalfEdge3D(v25, v20)),
                              new Vector3D(0.72932258655706560000, 0.18089143438393795000, -0.65982335037813940000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v19, v21), new HalfEdge3D(v21, v26), new HalfEdge3D(v26, v19)),
                              new Vector3D(0.47426239076215226000, 0.18089143438393765000, -0.86159936958837710000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v17, v19), new HalfEdge3D(v19, v26), new HalfEdge3D(v26, v17)),
                              new Vector3D(0.47540947348052090000, 0.25825185437921727000, -0.84100642817678230000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v23, v22), new HalfEdge3D(v22, v27), new HalfEdge3D(v27, v23)),
                              new Vector3D(0.64444609345549310000, -0.03079467299947462700, -0.76402939782760280000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v24, v23), new HalfEdge3D(v23, v27), new HalfEdge3D(v27, v24)),
                              new Vector3D(0.59069464800240810000, -0.03673568242502494000, -0.80605851056736510000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v21, v24), new HalfEdge3D(v24, v28), new HalfEdge3D(v28, v21)),
                              new Vector3D(0.53897082840409720000, 0.03169185683848829000, -0.84172802753593160000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v26, v21), new HalfEdge3D(v21, v28), new HalfEdge3D(v28, v26)),
                              new Vector3D(0.54630260082546940000, 0.09909601034401168000, -0.83170514550844720000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v20, v25), new HalfEdge3D(v25, v29), new HalfEdge3D(v29, v20)),
                              new Vector3D(0.68365121587975930000, 0.09909601034401440000, -0.72304978788464180000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v22, v20), new HalfEdge3D(v20, v29), new HalfEdge3D(v29, v22)),
                              new Vector3D(0.69509241200865510000, 0.03169185683848524400, -0.71822152918032120000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v17, v26), new HalfEdge3D(v26, v30), new HalfEdge3D(v30, v17)),
                              new Vector3D(0.59815244717004650000, 0.16223582843958295000, -0.78478862499083590000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v25, v17), new HalfEdge3D(v17, v30), new HalfEdge3D(v30, v25)),
                              new Vector3D(0.62606117175683520000, 0.16223582843958306000, -0.76271026293670700000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v22, v29), new HalfEdge3D(v29, v31), new HalfEdge3D(v31, v22)),
                              new Vector3D(0.66817412648012060000, 0.02856279682735566800, -0.74345645692261420000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v27, v22), new HalfEdge3D(v22, v31), new HalfEdge3D(v31, v27)),
                              new Vector3D(0.64470151510109850000, -0.01236073078256076800, -0.76433446131969560000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v24, v27), new HalfEdge3D(v27, v32), new HalfEdge3D(v32, v24)),
                              new Vector3D(0.59109635998463000000, 0.00022030614862306744, -0.80660092033056940000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v28, v24), new HalfEdge3D(v24, v32), new HalfEdge3D(v32, v28)),
                              new Vector3D(0.56966816951662990000, 0.02856852861135592000, -0.82137811987692630000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v26, v28), new HalfEdge3D(v28, v33), new HalfEdge3D(v33, v26)),
                              new Vector3D(0.56250064570981680000, 0.09747727538583728000, -0.82103057455821520000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v30, v26), new HalfEdge3D(v26, v33), new HalfEdge3D(v33, v30)),
                              new Vector3D(0.59225037826612680000, 0.11882058805681327000, -0.79694488974926150000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v29, v25), new HalfEdge3D(v25, v34), new HalfEdge3D(v34, v29)),
                              new Vector3D(0.66953530664389090000, 0.09747727538583957000, -0.73635633625346380000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v25, v30), new HalfEdge3D(v30, v34), new HalfEdge3D(v34, v25)),
                              new Vector3D(0.63924944602334990000, 0.11882058805681533000, -0.75976431451646430000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v27, v31), new HalfEdge3D(v31, v35), new HalfEdge3D(v35, v27)),
                              new Vector3D(0.63145956250466040000, 0.01006837356838149800, -0.77534343924167590000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v32, v27), new HalfEdge3D(v27, v35), new HalfEdge3D(v35, v32)),
                              new Vector3D(0.60693000454211280000, 0.01551796132205475800, -0.79460377702532860000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v29, v34), new HalfEdge3D(v34, v36), new HalfEdge3D(v36, v29)),
                              new Vector3D(0.65401484701798810000, 0.07317323334132480000, -0.75293443127699620000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v31, v29), new HalfEdge3D(v29, v36), new HalfEdge3D(v36, v31)),
                              new Vector3D(0.65731253388641850000, 0.04151924107504209600, -0.75247351143835530000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v13, v22), new HalfEdge3D(v22, v37), new HalfEdge3D(v37, v13)),
                              new Vector3D(0.71196751173775560000, -0.12395812189836788000, -0.69118495805781120000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v4, v13), new HalfEdge3D(v13, v37), new HalfEdge3D(v37, v4)),
                              new Vector3D(0.80021949912982260000, -0.39364322375455260000, -0.45243095120088683000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v9, v4), new HalfEdge3D(v4, v37), new HalfEdge3D(v37, v9)),
                              new Vector3D(0.76762702646460450000, -0.45645982940395010000, -0.44988128698760815000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v14, v9), new HalfEdge3D(v9, v37), new HalfEdge3D(v37, v14)),
                              new Vector3D(0.50329832707123220000, -0.26886500819707580000, -0.82122006876018640000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v23, v14), new HalfEdge3D(v14, v37), new HalfEdge3D(v37, v23)),
                              new Vector3D(0.52026253177009970000, -0.23377086174879438000, -0.82138789997990110000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v22, v23), new HalfEdge3D(v23, v37), new HalfEdge3D(v37, v22)),
                              new Vector3D(0.70376830665668830000, -0.12542118513427790000, -0.69927083227108110000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v28, v32), new HalfEdge3D(v32, v38), new HalfEdge3D(v38, v28)),
                              new Vector3D(0.58308932010150390000, 0.04702093739793623000, -0.81104616158994640000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v33, v28), new HalfEdge3D(v28, v38), new HalfEdge3D(v38, v33)),
                              new Vector3D(0.58209133982303130000, 0.07331680947456745000, -0.80981128514703860000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v30, v33), new HalfEdge3D(v33, v39), new HalfEdge3D(v39, v30)),
                              new Vector3D(0.60380052696739420000, 0.10469582237939637000, -0.79023016166823120000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v34, v30), new HalfEdge3D(v30, v39), new HalfEdge3D(v39, v34)),
                              new Vector3D(0.62854159213256590000, 0.10239851786794785000, -0.77100584336171320000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v35, v31), new HalfEdge3D(v31, v40), new HalfEdge3D(v40, v35)),
                              new Vector3D(0.63248791628264700000, 0.02852416248755342000, -0.77404483585307830000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v31, v36), new HalfEdge3D(v36, v40), new HalfEdge3D(v40, v31)),
                              new Vector3D(0.64317293425764520000, 0.04156053630654445000, -0.76459224326455430000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v32, v35), new HalfEdge3D(v35, v41), new HalfEdge3D(v41, v32)),
                              new Vector3D(0.60765366207917750000, 0.02855134939790428600, -0.79368875978517130000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v38, v32), new HalfEdge3D(v32, v41), new HalfEdge3D(v41, v38)),
                              new Vector3D(0.59678186631146750000, 0.04563039110124325000, -0.80110503147199700000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v34, v39), new HalfEdge3D(v39, v42), new HalfEdge3D(v42, v34)),
                              new Vector3D(0.62930295083642710000, 0.08969980240314648000, -0.77196615309053680000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v36, v34), new HalfEdge3D(v34, v42), new HalfEdge3D(v42, v36)),
                              new Vector3D(0.64425801589717930000, 0.07317757001702384000, -0.76129931840152940000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v35, v40), new HalfEdge3D(v40, v43), new HalfEdge3D(v43, v35)),
                              new Vector3D(0.62511669089738820000, 0.03704261067627682000, -0.77965182469855420000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v41, v35), new HalfEdge3D(v35, v43), new HalfEdge3D(v43, v41)),
                              new Vector3D(0.61482377708712000000, 0.03705578463116909000, -0.78779349575488100000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v33, v38), new HalfEdge3D(v38, v44), new HalfEdge3D(v44, v33)),
                              new Vector3D(0.59572042821351760000, 0.07471259943790275000, -0.79970944654564040000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v39, v33), new HalfEdge3D(v33, v44), new HalfEdge3D(v44, v39)),
                              new Vector3D(0.60480291553970440000, 0.08797886742447612000, -0.79150056995644640000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v36, v42), new HalfEdge3D(v42, v45), new HalfEdge3D(v45, v36)),
                              new Vector3D(0.63742891075119670000, 0.06466608844584133000, -0.76779077927756940000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v40, v36), new HalfEdge3D(v36, v45), new HalfEdge3D(v45, v40)),
                              new Vector3D(0.63699661940586580000, 0.05152464945097609500, -0.76914271586257170000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v38, v41), new HalfEdge3D(v41, v46), new HalfEdge3D(v46, v38)),
                              new Vector3D(0.60179210616596160000, 0.05156511108204621000, -0.79698638650571210000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v44, v38), new HalfEdge3D(v38, v46), new HalfEdge3D(v46, v44)),
                              new Vector3D(0.60233365024067230000, 0.06625174123861248000, -0.79549033970916270000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v40, v45), new HalfEdge3D(v45, v47), new HalfEdge3D(v47, v40)),
                              new Vector3D(0.63044878696050950000, 0.05156728769733743000, -0.77451606946503170000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v46, v41), new HalfEdge3D(v41, v48), new HalfEdge3D(v48, v46)),
                              new Vector3D(0.60861747580978950000, 0.05159378638178597300, -0.79178459782046210000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v39, v44), new HalfEdge3D(v44, v49), new HalfEdge3D(v49, v39)),
                              new Vector3D(0.61325353571030110000, 0.07998868170372875000, -0.78582562425649760000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v42, v39), new HalfEdge3D(v39, v49), new HalfEdge3D(v49, v42)),
                              new Vector3D(0.62658073914064200000, 0.08513540768181682000, -0.77469254526993960000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v49, v50), new HalfEdge3D(v50, v51), new HalfEdge3D(v51, v49)),
                              new Vector3D(0.61934069670637610000, 0.06751883799997335000, -0.78221372266049760000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v42, v49), new HalfEdge3D(v49, v51), new HalfEdge3D(v51, v42)),
                              new Vector3D(0.62616678688568520000, 0.07372601983969139000, -0.77619561258735190000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v45, v42), new HalfEdge3D(v42, v51), new HalfEdge3D(v51, v45)),
                              new Vector3D(0.63087436196130120000, 0.06610555663744055000, -0.77306377149726490000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v44, v46), new HalfEdge3D(v46, v52), new HalfEdge3D(v52, v44)),
                              new Vector3D(0.60705770798526970000, 0.06622657232030632000, -0.79189328845140130000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v49, v44), new HalfEdge3D(v44, v52), new HalfEdge3D(v52, v49)),
                              new Vector3D(0.61385754189323430000, 0.07431024289803456000, -0.78591151286975770000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v50, v49), new HalfEdge3D(v49, v52), new HalfEdge3D(v52, v50)),
                              new Vector3D(0.61873605845292790000, 0.06762791708438201000, -0.78268266545322380000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v51, v50), new HalfEdge3D(v50, v53), new HalfEdge3D(v53, v51)),
                              new Vector3D(0.62202619232673900000, 0.06358942918659198000, -0.78041002079369940000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v45, v51), new HalfEdge3D(v51, v53), new HalfEdge3D(v53, v45)),
                              new Vector3D(0.62783715913224560000, 0.06253187179595619000, -0.77582876114671420000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v47, v45), new HalfEdge3D(v45, v53), new HalfEdge3D(v53, v47)),
                              new Vector3D(0.62753979498019980000, 0.05613779846232104000, -0.77655801669934010000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v46, v48), new HalfEdge3D(v48, v54), new HalfEdge3D(v54, v46)),
                              new Vector3D(0.61128556235465760000, 0.05616419903316402000, -0.78941468443633170000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v52, v46), new HalfEdge3D(v46, v54), new HalfEdge3D(v54, v52)),
                              new Vector3D(0.61064199426201470000, 0.06228066874139325000, -0.78945390818263380000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v50, v52), new HalfEdge3D(v52, v54), new HalfEdge3D(v54, v50)),
                              new Vector3D(0.61631547751363840000, 0.06345390641173182000, -0.78493874534146100000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v53, v50), new HalfEdge3D(v50, v55), new HalfEdge3D(v55, v53)),
                              new Vector3D(0.62200596153346280000, 0.06012492435445458000, -0.78070069635437340000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v47, v53), new HalfEdge3D(v53, v55), new HalfEdge3D(v55, v47)),
                              new Vector3D(0.62466039416041250000, 0.05622848704293653000, -0.77886953285657580000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v54, v48), new HalfEdge3D(v48, v56), new HalfEdge3D(v56, v54)),
                              new Vector3D(0.61435425766350530000, 0.05617294702637999600, -0.78702823717646550000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v50, v54), new HalfEdge3D(v54, v56), new HalfEdge3D(v56, v50)),
                              new Vector3D(0.61664690394028640000, 0.06006120307977887000, -0.78494537882929720000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v50, v56), new HalfEdge3D(v56, v57), new HalfEdge3D(v57, v58), new HalfEdge3D(v58, v50)),
                              new Vector3D(0.61814798133946160000, 0.05840810932503987000, -0.78388874588874100000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v55, v50), new HalfEdge3D(v50, v58), new HalfEdge3D(v58, v59), new HalfEdge3D(v59, v55)),
                              new Vector3D(0.62068023505312020000, 0.05845953921829056000, -0.78188140282832450000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v43, v40), new HalfEdge3D(v40, v60), new HalfEdge3D(v60, v43)),
                              new Vector3D(0.62537589315963250000, 0.04311797882929476000, -0.77913146012497040000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v40, v47), new HalfEdge3D(v47, v60), new HalfEdge3D(v60, v40)),
                              new Vector3D(0.62660091517196470000, 0.04689607647157660000, -0.77792805009025630000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v43, v60), new HalfEdge3D(v60, v61), new HalfEdge3D(v61, v43)),
                              new Vector3D(0.61968141672966840000, 0.04869735565932542500, -0.78334124703841570000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v41, v43), new HalfEdge3D(v43, v61), new HalfEdge3D(v61, v41)),
                              new Vector3D(0.61426707595945210000, 0.04312776946694367500, -0.78791874891579460000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v48, v41), new HalfEdge3D(v41, v61), new HalfEdge3D(v61, v48)),
                              new Vector3D(0.61281832263446470000, 0.04691227395632742000, -0.78883010971674230000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v60, v62), new HalfEdge3D(v62, v61), new HalfEdge3D(v61, v60)),
                              new Vector3D(0.61963697196871950000, 0.05017380203695328000, -0.78328322627169360000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v62, v60), new HalfEdge3D(v60, v63), new HalfEdge3D(v63, v62)),
                              new Vector3D(0.62079679165336480000, 0.05137391270943935000, -0.78228643383725620000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v47, v55), new HalfEdge3D(v55, v63), new HalfEdge3D(v63, v47)),
                              new Vector3D(0.62354253437993170000, 0.05491260161539652400, -0.77985852178640740000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v60, v47), new HalfEdge3D(v47, v63), new HalfEdge3D(v63, v60)),
                              new Vector3D(0.62343582783812320000, 0.05111277107955967000, -0.78020205921310060000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v48, v61), new HalfEdge3D(v61, v64), new HalfEdge3D(v64, v48)),
                              new Vector3D(0.61576723260602060000, 0.05112371644902281000, -0.78626781751840960000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v56, v48), new HalfEdge3D(v48, v64), new HalfEdge3D(v64, v56)),
                              new Vector3D(0.61540464490699290000, 0.05499431541734822000, -0.78629049866997340000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v61, v62), new HalfEdge3D(v62, v64), new HalfEdge3D(v64, v61)),
                              new Vector3D(0.61840175306562480000, 0.05137616324917626000, -0.78418094956148730000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v56, v64), new HalfEdge3D(v64, v65), new HalfEdge3D(v65, v56)),
                              new Vector3D(0.61672226836911850000, 0.05489367903020470000, -0.78526449537730780000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v64, v62), new HalfEdge3D(v62, v65), new HalfEdge3D(v65, v64)),
                              new Vector3D(0.61822650055506770000, 0.05322648728293316600, -0.78419572497110370000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v62, v63), new HalfEdge3D(v63, v66), new HalfEdge3D(v66, v67), new HalfEdge3D(v67, v62)),
                              new Vector3D(0.62083818675211000000, 0.05328912667174397000, -0.78212544700253450000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v63, v55), new HalfEdge3D(v55, v66), new HalfEdge3D(v66, v63)),
                              new Vector3D(0.62229024458314360000, 0.05487563500085001400, -0.78086075338686640000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v65, v62), new HalfEdge3D(v62, v68), new HalfEdge3D(v68, v57), new HalfEdge3D(v57, v65)),
                              new Vector3D(0.61888428568868410000, 0.05472761383409186000, -0.78357330812862270000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v56, v65), new HalfEdge3D(v65, v57), new HalfEdge3D(v57, v56)),
                              new Vector3D(0.61754961517124200000, 0.05577502337217010000, -0.78455185906967600000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v66, v55), new HalfEdge3D(v55, v59), new HalfEdge3D(v59, v67), new HalfEdge3D(v67, v66)),
                              new Vector3D(0.62119836586231280000, 0.05557979209546433400, -0.78167990696999320000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v57, v68), new HalfEdge3D(v68, v69), new HalfEdge3D(v69, v58), new HalfEdge3D(v58, v57)),
                              new Vector3D(0.61900899274687500000, 0.05582846794347185000, -0.78339712091989720000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v59, v58), new HalfEdge3D(v58, v69), new HalfEdge3D(v69, v67), new HalfEdge3D(v67, v59)),
                              new Vector3D(0.62001897411297690000, 0.05580919464084649000, -0.78259939019490150000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v68, v62), new HalfEdge3D(v62, v67), new HalfEdge3D(v67, v69), new HalfEdge3D(v69, v68)),
                              new Vector3D(0.61909597982511180000, 0.05465114336548167000, -0.78341139913408860000),
                              constructionEpsilon));
         convexPolytope3D = new ConvexPolytope3D(faces, constructionEpsilon);
         troublesomePoint.set(0.60974689213119120000, 0.05344267959413851000, -0.77294417767419080000);
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190327_214757 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190327_214757()
      {
         double constructionEpsilon = 1.0E-10;
         Vertex3D v0 = new Vertex3D(0.04160085554696557600, -0.04404480651733777600, -4.95925734368968800000);
         Vertex3D v1 = new Vertex3D(-0.36103974833329210000, -0.62556710952389330000, -6.53132984259412700000);
         Vertex3D v2 = new Vertex3D(-1.35627922675442920000, 0.21226263279247082000, -1.23549588691827860000);
         Vertex3D v3 = new Vertex3D(-6.51875383795503800000, -0.11021427324055377000, -3.38696658460533760000);
         Vertex3D v4 = new Vertex3D(-5.76714510844884600000, 6.49893796580586350000, -0.33428079934875930000);
         Vertex3D v5 = new Vertex3D(-1.09725153429819230000, 2.36571354466364100000, -2.24365389768875370000);
         Vertex3D v6 = new Vertex3D(-3.03070832045488000000, 0.97936243471164540000, -0.62182617537840560000);
         Vertex3D v7 = new Vertex3D(-6.63931054903810600000, 2.37023283656933970000, -2.78158174112246040000);
         Vertex3D v8 = new Vertex3D(-6.61191506393674700000, 2.05742822043099150000, -8.31943254110272000000);
         List<Face3D> faces = new ArrayList<>();
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v1), new HalfEdge3D(v1, v2), new HalfEdge3D(v2, v0)),
                              new Vector3D(0.41997048407915440000, -0.88089479019584600000, 0.21828687800263452000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v2, v1), new HalfEdge3D(v1, v3), new HalfEdge3D(v3, v2)),
                              new Vector3D(-0.00317582225835749900, -0.98780260613829040000, 0.15567891783855890000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v4, v1), new HalfEdge3D(v1, v5), new HalfEdge3D(v5, v4)),
                              new Vector3D(0.48619644304692167000, 0.75364822195421310000, -0.44229783665961420000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v2, v4), new HalfEdge3D(v4, v5), new HalfEdge3D(v5, v2)),
                              new Vector3D(0.57114793308901320000, 0.29068391966684060000, 0.76765415219047040000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v2), new HalfEdge3D(v2, v5), new HalfEdge3D(v5, v0)),
                              new Vector3D(0.93615683035871000000, 0.05029805611260561000, 0.34796622612550590000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v1, v0), new HalfEdge3D(v0, v5), new HalfEdge3D(v5, v1)),
                              new Vector3D(0.55468760978201250000, 0.72408200090216700000, -0.40991085802138993000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v4, v2), new HalfEdge3D(v2, v6), new HalfEdge3D(v6, v4)),
                              new Vector3D(0.40059771236244980000, 0.15152860139140426000, 0.90363740283940050000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v3, v4), new HalfEdge3D(v4, v6), new HalfEdge3D(v6, v3)),
                              new Vector3D(-0.53143862005091870000, -0.30464834531285645000, 0.79041911592301060000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v2, v3), new HalfEdge3D(v3, v6), new HalfEdge3D(v6, v2)),
                              new Vector3D(-0.17588176657766583000, -0.81983870485382050000, 0.54491293085146140000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v4, v3), new HalfEdge3D(v3, v7), new HalfEdge3D(v7, v4)),
                              new Vector3D(-0.78848782037193010000, -0.18173130845956473000, 0.58758887723534840000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v1, v4), new HalfEdge3D(v4, v8), new HalfEdge3D(v8, v1)),
                              new Vector3D(0.45822515073207250000, 0.75528742951578280000, -0.46858362119477737000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v3, v1), new HalfEdge3D(v1, v8), new HalfEdge3D(v8, v3)),
                              new Vector3D(-0.26959452031091935000, -0.88346371181539640000, -0.38315879805073966000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v7, v3), new HalfEdge3D(v3, v8), new HalfEdge3D(v8, v7)),
                              new Vector3D(-0.99884472913058240000, -0.04800244743753618500, -0.00222982691247506850),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v4, v7), new HalfEdge3D(v7, v8), new HalfEdge3D(v8, v4)),
                              new Vector3D(-0.97617447718523620000, 0.21631635762048930000, -0.01704768365307238500),
                              constructionEpsilon));
         convexPolytope3D = new ConvexPolytope3D(faces, constructionEpsilon);
         troublesomePoint.set(-7.02417510790190200000, 3.02121058297256260000, -4.78310392726655000000);
      }
   }
}
