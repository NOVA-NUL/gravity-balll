package  {
	
	public class VectorResolution {
		
		public var v0_x:Number;
		public var v0_y:Number;
		public var v1_x:Number;
		public var v1_y:Number;
		public var slope_x:Number;
		public var slope_y:Number;
		
		//private var _slope_aim:Number;
		private var _v0_x_end:Number;
		private var _v0_y_end:Number;
		private var _v1_x_end:Number;
		private var _v1_y_end:Number;
		
		public function VectorResolution(v0_x_source:Number=0,v0_y_source:Number=0,v1_x_source:Number=0,v1_y_source:Number=0,slope_x_source:Number=0,slope_y_source:Number=0) {
			// constructor code
			
			v0_x=v0_x_source;
			v0_y=v0_y_source;
			v1_x=v1_x_source;
			v1_y=v1_y_source;
			slope_x=slope_x_source;
			slope_y=slope_y_source;
			//_slope_aim=Math.atan2(slope_y,slope_x);
			
		}
		
		public function setData(v0_x_source:Number,v0_y_source:Number,v1_x_source:Number,v1_y_source:Number,slope_x_source:Number,slope_y_source:Number){
			v0_x=v0_x_source;
			v0_y=v0_y_source;
			v1_x=v1_x_source;
			v1_y=v1_y_source;
			slope_x=slope_x_source;
			slope_y=slope_y_source;
			//_slope_aim=Math.atan2(slope_y,slope_x);
		}
		//数据初始化
		
		public function setSlope(slope_x_source:Number,slope_y_source:Number){
			slope_x=slope_x_source;
			slope_y=slope_y_source;
		}
		
		/*
		该方法弃用.
		public function vectorTransform(v_x:Number,v_y:Number){
			var slope_cosX:Number;
			var slope_sinX:Number;
			var slope_cosY:Number;
			var slope_sinY:Number;
			var slope_cossinX:Number;
			var slope_cossinY:Number;
			var v_aim:Array=new Array(false,false,false,false);
			
			slope_cosX=Math.cos(_slope_aim);
			slope_sinX=Math.sin(_slope_aim);
			slope_cosY=slope_sinX;
			slope_sinY=-slope_cosX;
			
			slope_cossinX=slope_cosX*slope_sinX;
			slope_cossinY=slope_cosY*slope_sinY;
			
			v_aim[0]=v_x*slope_cosX*slope_cosX+v_y*slope_cossinY;
			v_aim[1]=v_x*slope_cossinX+v_y*slope_sinY*slope_sinY;
			v_aim[2]=v_x*slope_sinX*slope_sinX+v_y*slope_cossinY;
			v_aim[3]=v_x*slope_cossinX+v_y*slope_cosY*slope_cosY;
			return v_aim;
		}
		//把矢量分解成为分别垂直和平行slope的两个矢量，以坐标形式表示,三角函数法
		*/
		
		public function vectorToSeparation(coor0_x:Number,coor0_y:Number,coor1_x:Number,coor1_y:Number,anti_intersect:Number=1):Array{
			var v_changed:Array=new Array(false,false,false,false);
			var v_transformer:Number;
			if( slope_x==0 ){
				if(slope_y>0){
					v_changed[1]=-anti_intersect;
					v_changed[3]=anti_intersect;
				}
				else{
					v_changed[1]=anti_intersect;
					v_changed[3]=-anti_intersect;
				}
				v_changed[0]=0;
				v_changed[2]=0;
			}
			else if( slope_y==0 ){
				if(slope_x>0){
					v_changed[0]=-anti_intersect;
					v_changed[2]=anti_intersect;
				}
				else{
					v_changed[0]=anti_intersect;
					v_changed[2]=-anti_intersect;
				}
				v_changed[1]=0;
				v_changed[3]=0;
			}
			else{
				v_transformer=1/(slope_x*slope_x+slope_y*slope_y);
				v_changed[0]=-Math.abs(v0_x)*v_transformer-anti_intersect;
				v_changed[1]=-Math.abs(v0_y)*v_transformer-anti_intersect;
				v_changed[2]=Math.abs(v1_x)*v_transformer+anti_intersect;
				v_changed[3]=Math.abs(v1_y)*v_transformer+anti_intersect;
				//通用
			}
			return v_changed;
		}
		//对慢速或相对速度较低的碰撞重叠的处理
		
		
		public function vectorTransform2(v_x:Number,v_y:Number):Array{
			var v_aim:Array=new Array(false,false,false,false);
			var k_aim:Number;
			//var k_ver:Number;
			if(slope_x==0){
				v_aim[0]=0;
				v_aim[1]=v_y;
				v_aim[2]=v_x;
				v_aim[3]=0;
			}
			else if( (k_aim=slope_y/slope_x)==0 ){
				v_aim[0]=v_x;
				v_aim[1]=0;
				v_aim[2]=0;
				v_aim[3]=v_y;
			}
			else{
				v_aim[0]=(v_y+v_x/k_aim)/(k_aim+1/k_aim);
				v_aim[1]=k_aim*v_aim[0];
				v_aim[2]=v_x-v_aim[0];
				v_aim[3]=v_y-v_aim[1];
			}//解方程
			return v_aim;
		} 
		//斜率方程法,把矢量分解成为分别平行和垂直slope的两个矢量，以坐标形式表示
		
		public function vectorSortDrag(v_x:Number,v_y:Number):Array{
			var v_all:Array=new Array();
			var v_answer:Array=new Array();
			v_all=vectorTransform2(v_x,v_y);
			v_all[0]=-v_all[0];
			v_all[1]=-v_all[1];
			v_answer[0]=v_all[0]+v_all[2];
			v_answer[1]=v_all[1]+v_all[3];
			return v_answer;
		}
		//其中一个ball在drag时另一个ball_v的反应
		
		public function vectorSort():Array{
			var v0_all:Array=new Array();
			var v1_all:Array=new Array();
			var v_answer:Array=new Array();
			v0_all=vectorTransform2(v0_x,v0_y);
			v1_all=vectorTransform2(v1_x,v1_y);
			v_answer[0]=v1_all[0]+v0_all[2];
			v_answer[1]=v1_all[1]+v0_all[3];
			v_answer[2]=v0_all[0]+v1_all[2];
			v_answer[3]=v0_all[1]+v1_all[3];
			/*如果上级出现NaN的反响
			if(v_answer[0]==NaN || v_answer[1]==NaN || v_answer[2]==NaN || v_answer[3]==NaN){
				v_answer[0]=v0_x;
				v_answer[1]=v0_y;
				v_answer[2]=v1_x;
				v_answer[3]=v1_y;
			}
			*/
			return v_answer;
		}
		//碰撞速度交换组合
		
	}
	
}
