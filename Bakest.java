package com.example.sensemart;

import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.bumptech.glide.Glide;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

// 'Bakest' 클래스는 'AppCompatActivity'를 상속받음
// 'AppCompatActivity'는 기본 Activity Class보다 더 많은 기능을 제공하는 서포트 라이브러리 클래스
public class Bakest extends AppCompatActivity {

    RecyclerView bakestRecyclerView; // RecyclerView 객체 선언
    BakestAdapter adapter; // BakestAdapter 객체 선언 → RecyclerView에 데이터를 연결, 각 제품 항목의 UI 관리
    static DatabaseReference databaseReference; // "products" 경로에 대한 참조
    List<Product> productList; // 'Firebase'에서 가져온 제품 목록을 저장하고 RecyclerView에 표시하기 위해 사용
    String coordinate;
    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate (savedInstanceState);
        setContentView(R.layout.basket_layout);

        /* "뒤로 가기" 버튼 설정 */
        ImageButton back_btn = findViewById(R.id.back_btn); // ImageButton 타입의 'back_btn' 변수를 선언하여 XML 파일에서 ID 참조
        back_btn.setOnClickListener(new View.OnClickListener() { // 버튼에 클릭 이벤트 설정
            @Override
            public void onClick(View v) {finish();} // 'finish()' 메서드를 호출하여 현재 액티비티 종료
        });

        /* 선택된 상품 DB 삭제 */
        Button selectProductDelete_btn = findViewById(R.id.click_delete_btn);
        selectProductDelete_btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                for (Product product : productList) { // productList에 있는 것들을 product에 넣고 하나씩 반복문 실행
                    if (product.isChecked()) { // product의 isChecked가 ture면 실행
                        DatabaseReference productRef = databaseReference.child(product.getName());
                        productRef.removeValue(); // DB 상에서 삭제

                        coordinate = product.getCoordinate();
                        removeWaypoints(coordinate);
                    }
                }
            }
        });

        /* "전체 선택" 체크 박스 설정 */
        CheckBox all_check = findViewById(R.id.all_click_checkbox);
        all_check.setOnCheckedChangeListener((buttonView, isChecked) -> {
            for (Product product : productList){ // productList에 있는 것들을 product에 넣고 하나씩 반복문 실행
                product.setChecked(isChecked); // product의 checkBox를 true로 설정
                DatabaseReference productRef = databaseReference.child(product.getName()); // DB의 각각 products/제품 이름에 참조하여 productRef에 할당
                productRef.child("checked").setValue(isChecked); // 할당된 제품의 자식들중 "checked"의 상태 변경
            }
            adapter.notifyDataSetChanged(); // adapter에게 데이터가 변경되었음을 알려서 RecyclerView를 업데이트
        });

        Button route_guide_start_button = findViewById(R.id.route_guide_start_btn);
        route_guide_start_button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(getApplicationContext(),RouteGuide.class);
                startActivity(intent);
            }
        });

        bakestRecyclerView = findViewById(R.id.bakest_recyclerview); // RecyclerView를 'bakestRecyclerView' 변수에 ID 할당 → 장바구니 목록을 표시
        bakestRecyclerView.setLayoutManager(new LinearLayoutManager(this)); // LayoutManager를 설정하여 아이템 배치 설정 → 세로 방향

        adapter = new BakestAdapter(this); // 'adapter' 변수를 통해 BakestAdapter 객체를 생성하여 할당
        bakestRecyclerView.setAdapter(adapter); // RecyclerView에 Adapter을 설정하여 데이터 표시

        productList = new ArrayList<>(); // 장바구니에 담긴 제품 목록을 담기위한 ArrayList
        databaseReference = FirebaseDatabase.getInstance().getReference("products"); // "products" 경로에 대한 참조를 얻어옴

        retrieveProducts();
    }

    public void removeWaypoints(String coordinate) {
        DatabaseReference waypointsRef = FirebaseDatabase.getInstance().getReference("admin/marker/waypoints");

        waypointsRef.addListenerForSingleValueEvent(new ValueEventListener() {
            @Override
            public void onDataChange(DataSnapshot dataSnapshot) {
                List<String> currentWaypoints = new ArrayList<>(); // currentWaypoints 초기화

                // 현재 저장된 데이터를 가져오기
                if (dataSnapshot.exists()) {
                    String currentData = dataSnapshot.getValue(String.class);
                    if (currentData != null && !currentData.isEmpty()) {
                        // 괄호 제거 및 리스트로 변환
                        currentWaypoints = new ArrayList<>(Arrays.asList(currentData.replace("[", "").replace("]", "").split(", ")));
                    }
                }

                // 좌표가 존재하면 삭제
                if (currentWaypoints.contains(coordinate)) {
                    currentWaypoints.remove(coordinate); // 좌표 삭제

                    // 삭제된 좌표가 있을 경우 전체 데이터를 문자열로 변환
                    String finalWaypointsString;

                    if (currentWaypoints.isEmpty()) {
                        // 웨이포인트가 없으면 대괄호만 포함한 문자열로 설정
                        finalWaypointsString = "[]";
                    } else {
                        // List의 toString() 메서드 사용하여 문자열로 변환
                        finalWaypointsString = currentWaypoints.toString(); // 대괄호는 toString() 메서드에서 자동 추가됨
                    }

                    // Firebase에 설정
                    waypointsRef.setValue(finalWaypointsString);
                    Log.d("삭제", "좌표값 삭제: " + coordinate); // 로그 추가
                } else {
                    Log.d("삭제", "삭제할 좌표가 존재하지 않습니다: " + coordinate);
                }
            }

            @Override
            public void onCancelled(DatabaseError databaseError) {
                System.err.println("Failed to read data: " + databaseError.getMessage());
            }
        });
    }

    private void retrieveProducts() {
        databaseReference.addValueEventListener(new ValueEventListener() { // "products" 경로에 대한 ValueEventListener을 추가하여 DB 값을 읽고 변화에 대한 알림을 받도록 함
            @Override
            public void onDataChange(@NonNull DataSnapshot snapshot) { // DB에서 데이터가 변경될 때마다 호출 → 데이터의 변경 사항을 실시간으로 수신하기 위함
                productList.clear(); // 중복된 리스트가 추가되지 않기 위해 productList 초기화
                for (DataSnapshot dataSnapshot : snapshot.getChildren()) {
                    // 'snapshot'은 "products" 아래에 있는 모든 데이터
                    // for문이 반복할 때마다 'snapshot.getChildren()'이 반환한 자식 데이터를 'dataSnapshot' 변수에 하나씩 대입하여 반복
                    Product product = dataSnapshot.getValue(Product.class); // 'Product' 클래스에 정의된 필드와 일치하는 속성들을 "제품 name" 데이터에서 가져와서 이를 Product 객체로 변환 (이름,가격,위치)
                    if (product != null) {  // 'product' 객체가 null이 아닌지 확인 → null인 경후 데이터 변환 실패
                        productList.add(product); // 'product' 객체를 'productList"에 추가
                    }
                }
                adapter.updateProducts(productList); // Adapter에 'productList'를 전달하여 RecyclerView 갱신
            }
            // 로그에 오류 메시지 출력
            @Override
            public void onCancelled(@NonNull DatabaseError error) {
                Log.e("Bakest", "Database error: " + error.getMessage());
            }
        });
    }

    public static class BakestAdapter extends RecyclerView.Adapter<BakestAdapter.ViewHolder> {
        private final List<Product> productList; // RecyclerView의 각 아이템에 대한 데이터를 저장하는 데 사용
        // 'Bakest' 클래스 내에 선언한 productList는 Firebase에서 가져온 제품 목록을 가져와, RecyclerView에 표시하기 위해 사용
        private final LayoutInflater inflater; // 'LayoutInflater inflater'은 XML 레이아웃 파일을 로드하고 실제 뷰 객체에 변환
        public BakestAdapter(Context context) { // Adapter의 인스턴스를 생성할 때 호출되며, 'Context context'는 안드로이드 애플리케이션의 실행 상태에 대한 정보를 제공하는 객체
            // 제품 목록에 담을 ArrayList와 LayoutInflater를 초기화
            this.productList = new ArrayList<>();
            this.inflater = LayoutInflater.from(context);
        }

        // RecyclerView에서 새로운 ViewHolder 객체를 생성하기 위해 호출 → LayoutInflation을 통해 새로운 ViewHolder을 생성하고 반환
        // 화면에 한 번에 10개의 아이템이 표시 할 수 있는 RecyclerView의 경우, DB에서 20개의 아이템을 가져온 경우
        // 함수는 처음에 10개의 아이템을 표시하기 위해 한 번 호출되고, 스크롤하여 나머지 10개의 아이템을 표시할 때 다시 호출
        @NonNull // 이 메서드는 항상 비어있지 않은 ViewHolder 객체를 반환
        @Override
        public ViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
            // 'bakest_frame'이라는 XML 레이아웃 파일을 inflate하여 새로운 뷰 객체 생성 → 이 뷰를 통해 RecylcerView의 각 아이템을 나타냄
            View view = inflater.inflate(R.layout.bakest_frame, parent, false);
            // RecyclerView의 새로운 아이템에 대한 ViewHolder를 생성하여 반환 → RecyclerView에 새로운 아이템을 표시하는 데 사용
            return new ViewHolder(view);
        }

        // RecyclerView의 각 아이템에 대한 데이터를 ViewHolder에 binding하고, 각 아이템의 상태를 업데이트
        @Override
        public void onBindViewHolder(@NonNull ViewHolder holder, int position) {
            Product product = productList.get(position); // RecyclerView에서 현재 위치에 해당하는 제품을 가져옴 → 이 위치는 아이템의 index
            holder.productName.setText(product.getName()); // ViewHolder 내의 TextView인 'productName'에 'product.getName()'을 이용하여 text 지정
            holder.productLocation.setText(product.getLocation());
            holder.productPrice.setText(product.getPrice());

            Glide.with(holder.itemView.getContext())
                    .load(product.getImageUrl())
                    .into(holder.productImage);

            holder.productCheckBox.setOnCheckedChangeListener(null);
            // 스크롤할 때 RecyclerView는 화면에 표시할 새로운 아이템을 로드하기 위해 onBindViewHolder()메서드 호출
            // 이 때, 기존의 리스너를 제거하지 않으면 이전 아이템의 상태가 혼란스러워질 수 있기 때문에 기존에 설정된 체크 변경 리스너 제거
            holder.productCheckBox.setChecked(product.isChecked()); // 현재 아이템의 CheckBox 상태를 해당 제품의 체크 상태로 설정

            /* CheckBox의 상태가 변경될 때 호출되는 체크 상태 변경 리스너 */
            // buttonView: CheckBox 자체를 나타내는 View, 이 매개변수를 통해 이벤트를 발생시킨 CheckBox 접근
            // isChecked: 체크 상태를 나타내는 boolean값, 이 매개변수를 통해 CheckBox가 현재 선택되었는 지 여부 확인
            holder.productCheckBox.setOnCheckedChangeListener((buttonView, isChecked) -> {
                product.setChecked(isChecked); // 제품의 체크 상태 변경
                DatabaseReference productRef = databaseReference.child(product.getName()); // 해당 제품의 참조 가져오기
                productRef.child("checked").setValue(isChecked); // 데이터베이스에 직접 업데이트
            });

            holder.deleteButton.setOnClickListener(new View.OnClickListener() { // 'X' 버튼을 눌렀을 때 리스너 호출
                @Override
                public void onClick(View v) {
                    int deleteProductposition = holder.getAdapterPosition(); // 버튼이 클릭된 ViewHolder의 위치를 가져와 변수에 할당, 이 위치는 RecyclerView에서의 아이템 인덱스
                    Product clickedProduct = productList.get(deleteProductposition); // productList에서 클릭 위치의 제품을 찾아음
                    removeProductData(clickedProduct.getName()); // removeProductData 메서드를 호출하여 제품의 이름을 인수로 전달하고 메서드를 통해 DB 상에서 제품 데이터 삭제
                    String coordinate = product.getCoordinate();
                    removeWaypoints(coordinate);
                }

                private void removeProductData(String Deletename) {
                    DatabaseReference productsRef = databaseReference.child(Deletename);
                    // DatabaseReference productRef로 경로에 있는 데이터에 접근 가능하며 데이터를 읽고 쓰는 작업 수행
                    // databaseReference.child(Deletename)은 products.꿀꽈배기
                    productsRef.removeValue(); // 지정된 경로에 있는 모든 데이터를 삭제
                }
            });
        }

        // RecyclerView에 의해 호출되어 표시할 아이템의 총 개수를 반환
        @Override
        public int getItemCount() {return productList.size();}
        // 'productList'는 현재 adpater가 관리하고 있는 데이터 리스트, productList의 크기가 RecyclerView에 표시될 아이템의 수
        // RecyclerView는 'getItemCount()' 메서드를 통해 adapter에 몇 개의 아이템이 있는지 확인하여 얼마나 렌더링할지 결정하는 데 사용
        // 만약 productList에 10개의 제품 객체가 있다면 메서드는 10을 반환

        public void updateProducts(List<Product> products) { // 새로운 제품 목록이 들어왔을 때 호출되는 메서드
            // 장바구니에 새로운 제품이 추가되거나 삭제될 때, 제품 목록 갱신할 때, DB로부터 새로운 데이터를 가져왔을 때 호출
            this.productList.clear(); // 현재 adapter에 저장된 'productList'를 모두 비우고, 새로운 데이터로 덮어쓸 준비
            this.productList.addAll(products); // 'products' 리스트의 모든 항목을 'productList'에 추가, 새로운 제품 목록이 adapter의 데이터리스트로 설정
            notifyDataSetChanged(); // RecyclerView에 데이터셋이 변경되었음을 알림
        }

        public void removeWaypoints(String coordinate) {
            DatabaseReference waypointsRef = FirebaseDatabase.getInstance().getReference("admin/marker/waypoints");

            waypointsRef.addListenerForSingleValueEvent(new ValueEventListener() {
                @Override
                public void onDataChange(DataSnapshot dataSnapshot) {
                    List<String> currentWaypoints = new ArrayList<>(); // currentWaypoints 초기화

                    // 현재 저장된 데이터를 가져오기
                    if (dataSnapshot.exists()) {
                        String currentData = dataSnapshot.getValue(String.class);
                        if (currentData != null && !currentData.isEmpty()) {
                            // 괄호 제거 및 리스트로 변환
                            currentWaypoints = new ArrayList<>(Arrays.asList(currentData.replace("[", "").replace("]", "").split(", ")));
                        }
                    }

                    // 좌표가 존재하면 삭제
                    if (currentWaypoints.contains(coordinate)) {
                        currentWaypoints.remove(coordinate); // 좌표 삭제

                        // 삭제된 좌표가 있을 경우 전체 데이터를 문자열로 변환
                        String finalWaypointsString;

                        if (currentWaypoints.isEmpty()) {
                            // 웨이포인트가 없으면 대괄호만 포함한 문자열로 설정
                            finalWaypointsString = "[]";
                        } else {
                            // List의 toString() 메서드 사용하여 문자열로 변환
                            finalWaypointsString = currentWaypoints.toString(); // 대괄호는 toString() 메서드에서 자동 추가됨
                        }

                        // Firebase에 설정
                        waypointsRef.setValue(finalWaypointsString);
                        Log.d("삭제", "좌표값 삭제: " + coordinate); // 로그 추가
                    }
                }

                @Override
                public void onCancelled(DatabaseError databaseError) {
                    System.err.println("Failed to read data: " + databaseError.getMessage());
                }
            });
        }


        public static class ViewHolder extends RecyclerView.ViewHolder {
            ImageView productImage;
            TextView productName;
            TextView productLocation;
            TextView productPrice;
            ImageButton deleteButton;
            CheckBox productCheckBox;

            public ViewHolder(View view) {
                super(view);
                productImage = view.findViewById(R.id.product_img);
                productName = view.findViewById(R.id.product_name);
                productLocation = view.findViewById(R.id.product_location);
                productPrice = view.findViewById(R.id.product_price);
                deleteButton = view.findViewById(R.id.delete_btn);
                productCheckBox = view.findViewById(R.id.product_checkbox);
            }
        }
    }
}