function clone(obj) {
    if (null == obj || "object" != typeof obj) return obj;
    var copy = obj.constructor();
    for (var attr in obj) {
        if (obj.hasOwnProperty(attr)) copy[attr] = obj[attr];
    }
    return copy;
}

const config = {
    apiKey: 'AIzaSyDzJl_MxEQSqme_RfpYG_XRpxxkA7iMlX8',
    authDomain: 'cpsmartcounter.firebaseapp.com',
    databaseURL: 'https://cpsmartcounter.firebaseio.com/',
};
firebase.initializeApp(config);

const db = firebase.database();

Vue.component('modal', {
    template: '#modal-template'
})

let vm = new Vue({
    el: '#app',
    data: {
        rooms: [{
            id: 'Loading...',
            count: 0,
            max: 0
        }],
        editingRoom: {
            id: '',
            max: 0
        },
        editingRoomError: '',
        isLoading: false
    },
    created() {
        this.isLoading = true;
        db.ref('/room').on('value', snapshot => {
            const value = snapshot.val();
            this.rooms = Object.keys(value).map(roomId => ({
                id: roomId,
                count: value[roomId].count,
                max: value[roomId].max
            }));
            this.isLoading = false;
        });
    },
    methods: {
        getStatus(count, max) {
            if (count < max / 3) return 'low';
            if (count > 2 * max / 3) return 'high';
            return 'medium';
        },
        clearCount(roomId) {
            this.isLoading = true;
            db.ref('/room/' + roomId + '/count').set(0).then(() => {
                this.isLoading = false;
            });
        },
        reset() {
            this.isLoading = true;
            db.ref('/room').once('value').then(snapshot => {
                const value = snapshot.val();
                Object.keys(value).forEach(key => value[key].count = 0);
                db.ref('/room').set(value).then(() => {
                    this.isLoading = false;
                });
            });
        },
        showEditRoomCapacity(roomId) {
            this.editingRoom = clone(this.rooms.filter(room => room.id === roomId)[0]);
        },
        editRoomCapacity() {
            const value = parseInt(this.editingRoom.max);
            if (isNaN(value) || value <= 0) {
                this.editingRoomError = 'Error setting capacity to ' + this.editingRoom.max;
                return;
            }
            this.isLoading = true;
            this.editingRoomError = ''
            db.ref('/room/' + this.editingRoom.id + '/max').set(value).then(() => {
                this.isLoading = false;
                $('#editroom-form').modal('toggle');
            });
        }
    }
});
